// Pathfinder - Geode mod for Geometry Dash 2.2081 (Windows + Android)
// Author: qemugd  https://github.com/qemugd/gd-sim
//
// What it does: plays the level with a physics look-ahead (cube mode), records
// every input with its exact 240 TPS physics frame, and exports a .gdr2 replay
// (GDReplayFormat v2) that Eclipse Menu and other GDR2 bots can play back.
//
// Credits
//   * Physics model: camila314's GD physics decompilations (camila314/gdp,
//     2.2 branch, used with permission). Constants and formulas below come from
//       - PlayerObject::updateTimeMod  (yStart / gravity / speed per speed portal)
//       - PlayerObject::updateJump     (jump = flip*yStart*v16, v16 = 0.8 if mini,
//                                       vel -= gravity*gravityMod*dt*flip, fall clamp 15)
//       - GJBaseGameLayer::update      (fixed 240 Hz step, dt per step = 0.25 frame units)
//   * Replay format: maxnut/GDReplayFormat (gdr2)
//   * ImGui for Cocos2d: matcool/gd-imgui-cocos
//   * Built on Geode.

#include <Geode/Geode.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <Geode/modify/MenuLayer.hpp>
#include <Geode/modify/PauseLayer.hpp>
#include <imgui-cocos.hpp>
#include <gdr/gdr.hpp>
#include <filesystem>
#include <chrono>

using namespace geode::prelude;

// ================================================================= physics (gdp)
static constexpr double kDt = 0.25;   // one 240 Hz step, in 1/60 s "frame units"
static constexpr double kStepSeconds = 1.0 / 240.0;
static constexpr double kMiniJump = 0.8;     // v16 in updateJump
static constexpr double kFallClamp = 15.0;   // setYVelocity(max(v,-15))

struct SpeedProfile { double yStart, gravity, speedMult; };

// PlayerObject::updateTimeMod
static SpeedProfile profileFor(float speed) {
    auto eq = [&](float v) { return std::abs(speed - v) < 0.001f; };
    if (eq(0.7f)) return {10.620032, 0.940199, 5.980002};
    if (eq(1.1f)) return {11.420032, 0.957199, 5.870002};
    if (eq(1.3f) || eq(1.6f)) return {11.230032, 0.961199, 6.000002};
    return {11.1800318, 0.958199024, 5.77000189}; // 0.9 (1x) and fallback
}

struct SimParams {
    CCRect start;
    double vx;        // units per second
    double jumpVel;   // signed
    double gravity;   // signed (pulls toward floor)
    double flip;      // +1 / -1
    int    steps;
    float  margin;
};

// Steps the cube in 240 Hz increments. jumpAt = step index of the jump (-1: never).
static bool pathIsSafe(SimParams const& s, int jumpAt,
                       std::vector<CCRect> const& hazards,
                       std::vector<CCPoint>* outPath = nullptr) {
    double dx = 0, dy = 0, vy = 0;
    bool air = false, jumped = false;
    float w = s.start.size.width, h = s.start.size.height;

    for (int i = 0; i < s.steps; i++) {
        if (!jumped && i == jumpAt) { vy = s.jumpVel; air = true; jumped = true; }
        if (air) {
            dy += vy * kDt;
            vy -= s.gravity * kDt;
            if (vy * s.flip < -kFallClamp) vy = -kFallClamp * s.flip;
            if (dy * s.flip <= 0.0 && vy * s.flip < 0.0) { dy = 0; vy = 0; air = false; }
        }
        dx += s.vx * kStepSeconds;

        CCRect r(s.start.origin.x + (float)dx, s.start.origin.y + (float)dy, w, h);
        if (outPath && i % 6 == 0) outPath->push_back({r.getMidX(), r.getMidY()});

        for (auto const& hz : hazards) {
            CCRect t(hz.origin.x - s.margin, hz.origin.y - s.margin,
                     hz.size.width + 2 * s.margin, hz.size.height + 2 * s.margin);
            if (r.intersectsRect(t)) return false;
        }
    }
    return true;
}

// ================================================================= state
struct PFState {
    bool  enabled      = true;    // drive the cube with the look-ahead
    bool  record       = true;    // record inputs for .gdr2
    bool  autoSave     = true;    // save when the level is completed
    bool  overlay      = true;
    int   horizon      = 130;     // steps (240 Hz) to look ahead
    float margin       = 2.f;
    int   cooldown     = 24;      // steps between jumps

    // live info
    bool  inLevel      = false;
    float nextHazard   = -1.f;
    bool  jumpSafe     = true;
    int   jumps        = 0;
    bool  dirty        = false;   // run had a practice respawn -> may not replay cleanly
    std::string status = "Ready.";
};
static PFState g;

struct Runtime {
    std::vector<GameObject*> hazards;       // sorted by x
    std::vector<std::pair<uint64_t, bool>> inputs; // (frame, down)
    bool   held = false;
    int    releaseAt = -1;
    int    lastJump = -100000;
    CCDrawNode* draw = nullptr;
};
static Runtime rt;

static int progress(GJBaseGameLayer* l) { return (int)l->m_gameState.m_currentProgress; }

// ================================================================= .gdr2 export
struct PFReplay : gdr::Replay<PFReplay, gdr::Input<>> {
    PFReplay() : Replay("Pathfinder", 1) {}
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
        dir = eclipse->getSaveDir() / "replays";   // where Eclipse looks for replays
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
    rep.description = "Generated by Pathfinder";
    rep.gameVersion = 22081;
    rep.framerate   = 240.0;
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
    rep.duration = (float)(last / 240.0);

    auto bytes = unwrapBytes(rep.exportData());
    if (!bytes) { g.status = "Export failed (GDR could not serialize the replay)."; return false; }

    std::string name = pl->m_level->m_levelName;
    for (auto& c : name) if (!std::isalnum((unsigned char)c)) c = '_';
    auto stamp = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    auto path = replayDir() / fmt::format("{}_{}_{}.gdr2", name, (int)pl->m_level->m_levelID, stamp);

    auto res = file::writeBinary(path, *bytes);
    if (!res) { g.status = "Could not write file: " + path.string(); return false; }

    g.status = "Saved " + path.filename().string() + (g.dirty ? "  (had practice respawns)" : "");
    log::info("Pathfinder saved {} ({} inputs)", path.string(), rep.inputs.size());
    return true;
}

// ================================================================= UI
static void drawUI() {
    ImGui::SetNextWindowSize({400, 460}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos({40, 40}, ImGuiCond_FirstUseEver);
    ImGui::Begin("Pathfinder");

    ImGui::Checkbox("Drive cube with look-ahead", &g.enabled);
    ImGui::Checkbox("Record inputs (.gdr2)", &g.record);
    ImGui::Checkbox("Auto-save on level complete", &g.autoSave);
    ImGui::Checkbox("Show hazards + predicted arc", &g.overlay);
    ImGui::Separator();
    ImGui::SliderInt("Look-ahead (steps)", &g.horizon, 60, 300);
    ImGui::SliderFloat("Safety margin", &g.margin, 0.f, 8.f, "%.1f");
    ImGui::SliderInt("Jump cooldown (steps)", &g.cooldown, 8, 96);
    ImGui::Separator();

    if (g.inLevel) {
        if (g.nextHazard >= 0.f) ImGui::Text("Next hazard: %.0f units", g.nextHazard);
        else                     ImGui::Text("Next hazard: none in range");
        ImGui::Text("Jump now would be: %s", g.jumpSafe ? "safe" : "unsafe");
        ImGui::Text("Inputs recorded: %d  (jumps: %d)", (int)rt.inputs.size(), g.jumps);
        if (g.dirty) ImGui::TextColored({1.f, 0.7f, 0.2f, 1.f}, "Run has practice respawns");

        if (ImGui::Button("Save .gdr2 now")) {
            if (auto pl = PlayLayer::get()) saveReplay(pl);
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear inputs")) { rt.inputs.clear(); rt.held = false; g.status = "Cleared."; }
    } else {
        ImGui::TextDisabled("Enter a level to start.");
    }

    ImGui::Separator();
    ImGui::TextWrapped("%s", g.status.c_str());
    ImGui::TextDisabled("Saves to: %s", replayDir().string().c_str());
    ImGui::Separator();
    ImGui::TextWrapped("Cube mode, hazards only. Blocks/orbs/pads/portals are not modelled.");
    ImGui::TextDisabled("Physics: camila314/gdp (with permission)");
    ImGui::TextDisabled("Format: maxnut/GDReplayFormat");
    ImGui::End();
}

static void toggleUI() { ImGuiCocos::get().toggle(); }

$on_mod(Loaded) {
    ImGuiCocos::get()
        .setup([] {
            auto& io = ImGui::GetIO();
            io.IniFilename = nullptr;
#ifdef GEODE_IS_ANDROID
            io.FontGlobalScale = 1.8f;
            ImGui::GetStyle().ScaleAllSizes(1.6f);
#endif
        })
        .draw([] { drawUI(); });
}

// ================================================================= entry buttons
class $modify(PFMenuLayer, MenuLayer) {
    bool init() {
        if (!MenuLayer::init()) return false;
        if (auto m = typeinfo_cast<CCMenu*>(this->getChildByID("bottom-menu"))) {
            if (auto spr = CCSprite::create("logo.png"_spr)) {
                spr->setScale(0.3f);
                auto btn = CCMenuItemExt::createSpriteExtra(spr, [](CCObject*) { toggleUI(); });
                btn->setID("pathfinder-button"_spr);
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

// ================================================================= decision logic
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
    double reach = s.vx * s.steps * kStepSeconds + 80.0;
    for (auto h : rt.hazards) {
        float dx = h->getPositionX() - px;
        if (dx < -40.f) continue;
        if (dx > reach) break;
        if (std::abs(h->getPositionY() - p->getPositionY()) > 220.f) continue;
        out.push_back(h->getObjectRect());
        if (g.nextHazard < 0.f) g.nextHazard = dx;
        if (draw && rt.draw) rt.draw->drawDot(h->getPosition(), 7.f, {1.f, 0.2f, 0.2f, 0.7f});
    }
    return out;
}

// Runs once per 240 Hz physics step, right before the game processes input.
static void tick(PlayLayer* pl) {
    int prog = progress(pl);

    // release the tap one step after pressing
    if (rt.releaseAt >= 0 && prog >= rt.releaseAt) {
        pl->handleButton(false, 1, true);
        rt.releaseAt = -1;
    }

    auto p = pl->m_player1;
    if (!g.enabled || !p || p->m_isDead) return;
    if (!isCubeLike(p) || !p->m_isOnGround) return;
    if (prog - rt.lastJump < g.cooldown) return;

    SimParams s = buildParams(p);
    auto near = nearHazards(p, s, false);
    if (near.empty()) return;
    if (pathIsSafe(s, -1, near)) return;        // standing still survives

    bool doJump;
    if (pathIsSafe(s, 0, near)) {
        doJump = true;                           // jumping now clears it
    } else {
        bool laterSafe = false;                  // wait for a better moment?
        for (int k = 1; k < s.steps / 2; k++) {
            if (pathIsSafe(s, k, near)) { laterSafe = true; break; }
        }
        doJump = !laterSafe;                     // nothing works: jump anyway
    }

    if (doJump) {
        pl->handleButton(true, 1, true);         // recorded by our handleButton hook
        rt.releaseAt = prog + 1;
        rt.lastJump = prog;
        g.jumps++;
    }
}

// ================================================================= hooks
class $modify(PFBaseGameLayer, GJBaseGameLayer) {
    void processCommands(float dt) {
        auto pl = PlayLayer::get();
        if (pl && static_cast<GJBaseGameLayer*>(pl) == this) tick(pl);
        GJBaseGameLayer::processCommands(dt);
    }

    // Same place every bot records from, so frames match what Eclipse replays.
    void handleButton(bool down, int button, bool isPlayer1) {
        GJBaseGameLayer::handleButton(down, button, isPlayer1);

        auto pl = PlayLayer::get();
        if (!g.record || !pl || static_cast<GJBaseGameLayer*>(pl) != this) return;
        if (button != 1 || !m_player1 || m_player1->m_isDead) return;
        if (!isPlayer1 && !m_gameState.m_isDualMode) return;   // GDR2: P2 only in 2P levels
        if (!isPlayer1) return;                                // this mod only drives P1
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

        rt.hazards.clear();
        for (auto obj : CCArrayExt<GameObject*>(m_objects)) {
            if (obj && obj->m_objectType == GameObjectType::Hazard) rt.hazards.push_back(obj);
        }
        std::sort(rt.hazards.begin(), rt.hazards.end(),
                  [](GameObject* a, GameObject* b) { return a->getPositionX() < b->getPositionX(); });

        rt.draw = CCDrawNode::create();
        m_objectLayer->addChild(rt.draw, 1000);
        g.status = "Level loaded. Play (or let the assist play) to record.";
    }

    void resetLevel() {
        PlayLayer::resetLevel();
        bool fromCheckpoint = m_isPracticeMode && m_checkpointArray && m_checkpointArray->count() > 0;
        if (fromCheckpoint) {
            // keep inputs up to the restored frame; the run may not replay cleanly
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

