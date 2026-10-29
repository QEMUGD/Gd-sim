#include <Geode/Geode.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <Geode/modify/MenuLayer.hpp>
#include <Geode/modify/PauseLayer.hpp>
#include <Geode/modify/LevelSelectLayer.hpp>
#include <Geode/modify/LevelInfoLayer.hpp>
#include <imgui-cocos.hpp>
#include <gdr/gdr.hpp>
#include <filesystem>
#include <chrono>
#include <thread>
#include <algorithm>

using namespace geode::prelude;

// ================================================================= Config & Physics (Up to 5000 TPS, 10x Speed, 16 CPS Lock)
struct SimConfig {
    int tps = 240;                  // Default 240 TPS, adjustable up to 5000 TPS
    float speedMultiplier = 1.0f;   // Adjustable speed up to 10x
    bool cpsLimitEnabled = true;    // 16 CPS lock enabled by default
    int maxCps = 16;
};
static SimConfig cfg;

static double getDt() { return 1.0 / (double)cfg.tps; }
static constexpr double kMiniJump = 0.8;     
static constexpr double kFallClamp = 15.0;   

struct SpeedProfile { double yStart, gravity, speedMult; };

static SpeedProfile profileFor(float speed) {
    auto eq = [&](float v) { return std::abs(speed - v) < 0.001f; };
    if (eq(0.7f)) return {10.620032, 0.940199, 5.980002};
    if (eq(1.1f)) return {11.420032, 0.957199, 5.870002};
    if (eq(1.3f) || eq(1.6f)) return {11.230032, 0.961199, 6.000002};
    return {11.1800318, 0.958199024, 5.77000189};
}

struct SimParams {
    CCRect start;
    double vx;        
    double jumpVel;   
    double gravity;   
    double flip;      
    int    steps;
    float  margin;
};

// Look-ahead simulation using variable TPS
static bool pathIsSafe(SimParams const& s, int jumpAt,
                       std::vector<CCRect> const& hazards,
                       std::vector<CCPoint>* outPath = nullptr) {
    double dx = 0, dy = 0, vy = 0;
    bool air = false, jumped = false;
    float w = s.start.size.width, h = s.start.size.height;
    double dtStep = getDt();

    for (int i = 0; i < s.steps; i++) {
        if (!jumped && i == jumpAt) { vy = s.jumpVel; air = true; jumped = true; }
        if (air) {
            dy += vy * dtStep;
            vy -= s.gravity * dtStep;
            if (vy * s.flip < -kFallClamp) vy = -kFallClamp * s.flip;
            if (dy * s.flip <= 0.0 && vy * s.flip < 0.0) { dy = 0; vy = 0; air = false; }
        }
        dx += s.vx * dtStep;

        CCRect r(s.start.origin.x + (float)dx, s.start.origin.y + (float)dy, w, h);
        if (outPath && i % std::max(1, cfg.tps / 40) == 0) outPath->push_back({r.getMidX(), r.getMidY()});

        for (auto const& hz : hazards) {
            CCRect t(hz.origin.x - s.margin, hz.origin.y - s.margin,
                     hz.size.width + 2 * s.margin, hz.size.height + 2 * s.margin);
            if (r.intersectsRect(t)) return false;
        }
    }
    return true;
}

// ================================================================= State & Mapping Logs
struct PFState {
    bool  enabled      = true;    
    bool  record       = true;    
    bool  autoSave     = true;    
    bool  overlay      = true;
    int   horizon      = 250;     
    float margin       = 2.f;
    int   cooldown     = 16;      

    bool  inLevel      = false;
    float nextHazard   = -1.f;
    bool  jumpSafe     = true;
    int   jumps        = 0;
    bool  dirty        = false;   
    int   attempts     = 1;
    std::string status = "Ready. Level Mapper & Bot active.";
    std::vector<std::string> matchLogs;
};
static PFState g;

struct Runtime {
    std::vector<GameObject*> hazards;       
    std::vector<std::pair<uint64_t, bool>> inputs; 
    bool   held = false;
    int    releaseAt = -1;
    int    lastJump = -100000;
    CCDrawNode* draw = nullptr;
    std::chrono::steady_clock::time_point lastClickTime = std::chrono::steady_clock::now();
};
static Runtime rt;

static int progress(GJBaseGameLayer* l) { return (int)l->m_gameState.m_currentProgress; }

// ================================================================= .gdr2 export
struct PFReplay : gdr::Replay<PFReplay, gdr::Input<>> {
    PFReplay() : Replay("Pathfinder Advanced", 1) {}
};

template <class R>
static std::optional<std::vector<uint8_t>> unwrapBytes(R&& r) {
    if constexpr (requires { r.isOk(); }) {
        if (r.isOk()) return std::vector<uint8_t>(std::move(r).unwrap());
        return std::nullopt;
    } else if constexpr (requires { r.has_value(); }) {
        if (r.has_value()) return std::vector<uint8_t>(*r);
        return std::nullopt;
    } else {
        return std::vector<uint8_t>(std::forward<R>(r));
    }
}

static std::filesystem::path replayDir() {
    std::filesystem::path dir;
    if (auto eclipse = Loader::get()->getLoadedMod("eclipse.eclipse-menu")) {
        dir = eclipse->getSaveDir() / "replays";   
    } else {
        dir = Mod::get()->getSaveDir() / "replays";
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    return dir;
}

static bool saveReplay(PlayLayer* pl) {
    if (rt.inputs.empty()) { g.status = "Nothing to save: no inputs recorded."; return false; }

    PFReplay rep;
    rep.author      = GJAccountManager::get()->m_username;
    rep.description = fmt::format("Generated by Pathfinder (TPS: {}, Speed: {}x, Attempt: {})", cfg.tps, cfg.speedMultiplier, g.attempts);
    rep.gameVersion = 22081;
    rep.framerate   = (double)cfg.tps;
    rep.seed        = 0;
    rep.coins       = 0;
    rep.ldm         = false;
    rep.platformer  = pl->m_level->isPlatformer();
    rep.levelInfo.id   = pl->m_level->m_levelID;
    rep.levelInfo.name = pl->m_level->m_levelName;

    uint64_t last = 0;
    for (auto const& [frame, down] : rt.inputs) {
        rep.inputs.push_back(gdr::Input<>(frame, 1, false, down));
        last = frame;
    }
    rep.duration = (float)(last / (double)cfg.tps);

    auto bytes = unwrapBytes(rep.exportData());
    if (!bytes) { g.status = "Export failed (GDR could not serialize replay)."; return false; }

    std::string name = pl->m_level->m_levelName;
    for (auto& c : name) if (!std::isalnum((unsigned char)c)) c = '_';
    auto stamp = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    auto path = replayDir() / fmt::format("{}_{}_tps{}_v{}.gdr2", name, (int)pl->m_level->m_levelID, cfg.tps, stamp);

    auto res = file::writeBinary(path, *bytes);
    if (!res) { g.status = "Could not write file: " + path.string(); return false; }

    g.status = "Saved replay: " + path.filename().string();
    log::info("Pathfinder saved replay to {}", path.string());
    return true;
}

// ================================================================= Cool GUI for All Platforms (PC & Mobile)
static void drawUI() {
    ImGui::SetNextWindowSize({460, 540}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos({40, 40}, ImGuiCond_FirstUseEver);
    ImGui::Begin("Pathfinder - AI Bot & Level Mapper");

    ImGui::TextColored({0.2f, 1.0f, 0.4f, 1.0f}, "Status: %s", g.status.c_str());
    ImGui::Separator();

    ImGui::Checkbox("Enable Pathfinder Bot", &g.enabled);
    ImGui::Checkbox("Record .gdr2 Replay", &g.record);
    ImGui::Checkbox("Auto-save on Level Complete", &g.autoSave);
    ImGui::Checkbox("Show Level Hazards & Predicted Arc", &g.overlay);
    
    ImGui::Separator();
    ImGui::Text("Performance & Speed Settings:");
    
    // TPS Slider up to 5000 TPS
    ImGui::SliderInt("Simulation TPS", &cfg.tps, 240, 5000);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Default is 240. Scale up to 5000 for ultra-high precision look-ahead.");

    // 10x Speed Multiplier Slider
    ImGui::SliderFloat("Game Speed Multiplier", &cfg.speedMultiplier, 0.5f, 10.0f, "%.1fx");

    // 16 CPS Lock (Enabled by default)
    ImGui::Checkbox("Lock Clicks Per Second (CPS)", &cfg.cpsLimitEnabled);
    if (cfg.cpsLimitEnabled) {
        ImGui::SliderInt("Max CPS", &cfg.maxCps, 1, 30);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Default limit is 16 CPS. Disable or adjust here before entering a level.");
    }

    ImGui::Separator();
    ImGui::SliderInt("Look-ahead Horizon", &g.horizon, 60, 600);
    ImGui::SliderFloat("Safety Margin", &g.margin, 0.f, 10.f, "%.1f");
    ImGui::SliderInt("Jump Cooldown", &g.cooldown, 4, 60);

    ImGui::Separator();
    if (g.inLevel) {
        ImGui::Text("Attempts: %d | Recorded Jumps: %d", g.attempts, g.jumps);
        if (ImGui::Button("Save Replay Now")) {
            if (auto pl = PlayLayer::get()) saveReplay(pl);
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Map & Inputs")) {
            rt.inputs.clear();
            rt.held = false;
            g.status = "Inputs and level map reset.";
        }
    } else {
        ImGui::TextDisabled("Enter a level to start mapping and bot execution.");
    }

    if (ImGui::CollTreeNode("Mapper & Bot Logs")) {
        for (auto const& logMsg : g.matchLogs) {
            ImGui::TextUnformatted(logMsg.c_str());
        }
    }

    ImGui::Separator();
    ImGui::TextDisabled("Cross-Platform GUI | camila314 physics | maxnut gdr2");
    ImGui::End();
}

static void toggleUI() { ImGuiCocos::get().toggle(); }

$on_mod(Loaded) {
    ImGuiCocos::get()
        .setup([] {
            auto& io = ImGui::GetIO();
            io.IniFilename = nullptr;
#ifdef GEODE_IS_ANDROID
            io.FontGlobalScale = 2.0f;
            ImGui::GetStyle().ScaleAllSizes(1.8f);
#else
            io.FontGlobalScale = 1.2f;
#endif
        })
        .draw([] { drawUI(); });
}

// ================================================================= Level Pages Integration with Geometry Dash Select Cube Button
class $modify(PFLevelSelectLayer, LevelSelectLayer) {
    bool init(int p0) {
        if (!LevelSelectLayer::init(p0)) return false;
        
        // Add Pathfinder button with Geometry Dash select cube icon on Level Select page
        auto menu = this->getChildByID("right-menu");
        if (!menu) menu = CCMenu::create();
        
        if (auto spr = CCSprite::createWithSpriteFrameName("GJ_selectSongBtn_001.png")) {
            spr->setScale(0.9f);
            auto btn = CCMenuItemExt::createSpriteExtra(spr, [](CCObject*) {
                toggleUI();
            });
            btn->setID("pathfinder-select-btn"_spr);
            this->addChild(btn, 100);
            btn->setPosition({30.f, 30.f});
        }
        return true;
    }
};

class $modify(PFLevelInfoLayer, LevelInfoLayer) {
    bool init(GJGameLevel* level, bool p1) {
        if (!LevelInfoLayer::init(level, p1)) return false;

        // Add Pathfinder button on Level Info page
        if (auto menu = this->getChildByID("other-menu")) {
            if (auto spr = CCSprite::createWithSpriteFrameName("GJ_optionsBtn_001.png")) {
                spr->setScale(0.8f);
                auto btn = CCMenuItemExt::createSpriteExtra(spr, [](CCObject*) {
                    toggleUI();
                });
                btn->setID("pathfinder-info-btn"_spr);
                menu->addChild(btn);
                menu->updateLayout();
            }
        }
        return true;
    }
};

class $modify(PFMenuLayer, MenuLayer) {
    bool init() {
        if (!MenuLayer::init()) return false;
        if (auto m = typeinfo_cast<CCMenu*>(this->getChildByID("bottom-menu"))) {
            if (auto spr = CCSprite::create("logo.png"_spr)) {
                spr->setScale(0.3f);
                auto btn = CCMenuItemExt::createSpriteExtra(spr, [](CCObject*) { toggleUI(); });
                btn->setID("pathfinder-main-btn"_spr);
                m->addChild(btn);
                m->updateLayout();
            }
        }
        return true;
    }
};

class $modify(PFPauseLayer, PauseLayer) {
    void customSetup() {
        PauseLayer::customSetup();
        auto winSize = CCDirector::get()->getWinSize();
        auto menu = CCMenu::create();
        menu->setPosition({0, 0});
        if (auto spr = CCSprite::create("logo.png"_spr)) {
            spr->setScale(0.28f);
            auto btn = CCMenuItemExt::createSpriteExtra(spr, [](CCObject*) { toggleUI(); });
            btn->setPosition({winSize.width - 35.f, 35.f});
            menu->addChild(btn);
        }
        this->addChild(menu);
    }
};

// ================================================================= Decision & Bot Logic with Level Mapping and Failure Logging
static bool isCubeLike(PlayerObject* p) {
    return !(p->m_isShip || p->m_isBird || p->m_isBall || p->m_isDart ||
             p->m_isRobot || p->m_isSpider || p->m_isSwing);
}

static SimParams buildParams(PlayerObject* p) {
    auto prof = profileFor(p->m_playerSpeed);
    SimParams s;
    s.start   = p->getObjectRect();
    s.flip    = p->m_isUpsideDown ? -1.0 : 1.0;
    double v16 = (p->m_vehicleSize != 1.f) ? kMiniJump : 1.0;
    s.jumpVel = s.flip * prof.yStart * v16;
    s.gravity = prof.gravity * p->m_gravityMod * s.flip;
    s.vx      = p->m_playerSpeed * prof.speedMult * 60.0;
    s.steps   = g.horizon;
    s.margin  = g.margin;
    return s;
}

static std::vector<CCRect> nearHazards(PlayerObject* p, SimParams const& s, bool draw) {
    std::vector<CCRect> out;
    g.nextHazard = -1.f;
    float px = p->getPositionX();
    double reach = s.vx * s.steps * (1.0 / (double)cfg.tps) + 100.0;
    for (auto h : rt.hazards) {
        float dx = h->getPositionX() - px;
        if (dx < -40.f) continue;
        if (dx > reach) break;
        if (std::abs(h->getPositionY() - p->getPositionY()) > 250.f) continue;
        out.push_back(h->getObjectRect());
        if (g.nextHazard < 0.f) g.nextHazard = dx;
        if (draw && rt.draw) rt.draw->drawDot(h->getPosition(), 7.f, {1.f, 0.2f, 0.2f, 0.7f});
    }
    return out;
}

static void tick(PlayLayer* pl) {
    int prog = progress(pl);

    // Enforce 10x speed multiplier via game time scale
    if (pl && pl->m_timeMod != cfg.speedMultiplier) {
        pl->m_timeMod = cfg.speedMultiplier;
    }

    if (rt.releaseAt >= 0 && prog >= rt.releaseAt) {
        pl->handleButton(false, 1, true);
        rt.releaseAt = -1;
    }

    auto p = pl->m_player1;
    if (!g.enabled || !p || p->m_isDead) return;
    if (!isCubeLike(p) || !p->m_isOnGround) return;
    if (prog - rt.lastJump < g.cooldown) return;

    // 16 CPS Lock validation (enabled by default)
    if (cfg.cpsLimitEnabled) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - rt.lastClickTime;
        double minInterval = 1.0 / (double)cfg.maxCps;
        if (elapsed.count() < minInterval) return;
    }

    SimParams s = buildParams(p);
    auto near = nearHazards(p, s, false);
    if (near.empty()) return;
    if (pathIsSafe(s, -1, near)) return;        

    bool doJump;
    if (pathIsSafe(s, 0, near)) {
        doJump = true;                           
    } else {
        bool laterSafe = false;                  
        for (int k = 1; k < s.steps / 2; k++) {
            if (pathIsSafe(s, k, near)) { laterSafe = true; break; }
        }
        doJump = !laterSafe;                     
    }

    if (doJump) {
        pl->handleButton(true, 1, true);         
        rt.releaseAt = prog + 1;
        rt.lastJump = prog;
        g.jumps++;
        if (cfg.cpsLimitEnabled) {
            rt.lastClickTime = std::chrono::steady_clock::now();
        }
    }
}

// ================================================================= Hooks & Bot Logging
class $modify(PFBaseGameLayer, GJBaseGameLayer) {
    void processCommands(float dt) {
        auto pl = PlayLayer::get();
        if (pl && static_cast<GJBaseGameLayer*>(pl) == this) tick(pl);
        GJBaseGameLayer::processCommands(dt);
    }

    void handleButton(bool down, int button, bool isPlayer1) {
        GJBaseGameLayer::handleButton(down, button, isPlayer1);

        auto pl = PlayLayer::get();
        if (!g.record || !pl || static_cast<GJBaseGameLayer*>(pl) != this) return;
        if (button != 1 || !m_player1 || m_player1->m_isDead) return;
        if (!isPlayer1) return;                                
        if (down == rt.held) return;
        rt.held = down;
        rt.inputs.emplace_back((uint64_t)progress(this), down);
    }
};

class $modify(PFPlayLayer, PlayLayer) {
    void setupHasCompleted() {
        PlayLayer::setupHasCompleted();
        g.inLevel = true;
        g.jumps = 0;
        g.dirty = false;
        rt.inputs.clear();
        rt.held = false;
        rt.releaseAt = -1;
        rt.lastJump = -100000;

        // Map level hazards upon entering
        rt.hazards.clear();
        for (auto obj : CCArrayExt<GameObject*>(m_objects)) {
            if (obj && obj->m_objectType == GameObjectType::Hazard) rt.hazards.push_back(obj);
        }
        std::sort(rt.hazards.begin(), rt.hazards.end(),
                  [](GameObject* a, GameObject* b) { return a->getPositionX() < b->getPositionX(); });

        std::string mapMsg = fmt::format("[Attempt {}] Level mapped successfully: {} hazards detected (TPS: {}, Speed: {}x).", 
            g.attempts, rt.hazards.size(), cfg.tps, cfg.speedMultiplier);
        g.matchLogs.push_back(mapMsg);
        log::info("{}", mapMsg);

        rt.draw = CCDrawNode::create();
        m_objectLayer->addChild(rt.draw, 1000);
        g.status = "Mapper & Bot active.";
    }

    void resetLevel() {
        PlayLayer::resetLevel();
        
        // If bot fails/dies, log failure, increment attempt, and re-map/retry
        if (m_player1 && m_player1->m_isDead) {
            std::string failMsg = fmt::format("[Attempt {}] Bot failed at {}%. Logging failure, re-mapping, and retrying...", g.attempts, progress(this));
            g.matchLogs.push_back(failMsg);
            log::warn("{}", failMsg);
            g.attempts++;
        }

        bool fromCheckpoint = m_isPracticeMode && m_checkpointArray && m_checkpointArray->count() > 0;
        if (fromCheckpoint) {
            uint64_t now = (uint64_t)progress(this);
            std::erase_if(rt.inputs, [&](auto const& in) { return in.first > now; });
            g.dirty = true;
        } else {
            rt.inputs.clear();
            g.dirty = false;
            g.jumps = 0;
        }
        rt.held = false;
        rt.releaseAt = -1;
        rt.lastJump = -100000;
    }

    void levelComplete() {
        PlayLayer::levelComplete();
        std::string passMsg = fmt::format("[Attempt {}] Level PASSED successfully! Replay saved.", g.attempts);
        g.matchLogs.push_back(passMsg);
        log::info("{}", passMsg);
        if (g.record && g.autoSave && !rt.inputs.empty()) saveReplay(this);
    }

    void onQuit() {
        g.inLevel = false;
        rt.draw = nullptr;
        PlayLayer::onQuit();
    }

    void postUpdate(float dt) {
        PlayLayer::postUpdate(dt);
        if (rt.draw) rt.draw->clear();
        auto p = m_player1;
        if (!p || p->m_isDead || !isCubeLike(p)) return;

        SimParams s = buildParams(p);
        auto near = nearHazards(p, s, g.overlay);
        g.jumpSafe = pathIsSafe(s, 0, near);

        if (g.overlay && rt.draw) {
            std::vector<CCPoint> path;
            pathIsSafe(s, 0, near, &path);
            ccColor4F col = g.jumpSafe ? ccColor4F{0.2f, 1.f, 0.4f, 0.8f}
                                       : ccColor4F{1.f, 0.6f, 0.1f, 0.8f};
            for (auto const& pt : path) rt.draw->drawDot(pt, 2.5f, col);
        }
    }
};
