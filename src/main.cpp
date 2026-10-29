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
//     Per-vehicle models (ship/ufo/ball/wave/robot/spider/swing) are taken from the
//     same updateJump function (gravity factors, click velocities, clamps).
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
static constexpr double kFallClamp = 15.0;   // ground-vehicle fall clamp
static constexpr double kFlyGravity = 0.9582; // usedGravity for ball/flying/spider

struct SpeedProfile { double yStart, gravity, speedMult; };

// PlayerObject::updateTimeMod
static SpeedProfile profileFor(float speed) {
    auto eq = [&](float v) { return std::abs(speed - v) < 0.001f; };
    if (eq(0.7f)) return {10.620032, 0.940199, 5.980002};
    if (eq(1.1f)) return {11.420032, 0.957199, 5.870002};
    if (eq(1.3f) || eq(1.6f)) return {11.230032, 0.961199, 6.000002};
    return {11.1800318, 0.958199024, 5.77000189}; // 0.9 (1x) and fallback
}

enum class Mode { Cube, Ship, Ufo, Ball, Wave, Robot, Spider, Swing };

static Mode modeOf(PlayerObject* p) {
    if (p->m_isShip)   return Mode::Ship;
    if (p->m_isBird)   return Mode::Ufo;
    if (p->m_isBall)   return Mode::Ball;
    if (p->m_isDart)   return Mode::Wave;
    if (p->m_isRobot)  return Mode::Robot;
    if (p->m_isSpider) return Mode::Spider;
    if (p->m_isSwing)  return Mode::Swing;
    return Mode::Cube;
}
static char const* modeName(Mode m) {
    switch (m) {
        case Mode::Cube: return "Cube";   case Mode::Ship: return "Ship";
        case Mode::Ufo: return "UFO";     case Mode::Ball: return "Ball";
        case Mode::Wave: return "Wave";   case Mode::Robot: return "Robot";
        case Mode::Spider: return "Spider"; case Mode::Swing: return "Swing";
    }
    return "?";
}
static bool holdMode(Mode m)   { return m == Mode::Ship || m == Mode::Wave || m == Mode::Robot; }
static bool groundMode(Mode m) { return m == Mode::Cube || m == Mode::Ball || m == Mode::Robot || m == Mode::Spider; }

struct SimParams {
    CCRect start;
    Mode   mode;
    bool   mini;
    double vx;          // units per second
    double yStart;      // base jump velocity for this speed
    double jumpScale;   // 0.8 when mini (ground vehicles)
    double gBase;       // profile gravity * gravityMod
    double gAlt;        // 0.9582 * gravityMod
    double waveV;       // wave vertical speed (playerSpeed * speedMult)
    double flip0;
    double vy0;
    bool   grounded0;
    bool   held0;
    double surfDown, surfUp;   // landing surfaces, relative to start y
    double floorRel, ceilRel;  // level bounds, relative to start y
    int    steps;
    float  margin;
};

// kind: 0 keep current input, 1 click at a, 2 toggle hold at a (and back after b),
//       3 hold from a for b steps, 4 click at a then every b steps
struct Policy { int kind, a, b; };

// Simulates the vehicle in 240 Hz steps and returns how many steps it survives
// (== s.steps means no hazard was touched).
static int survive(SimParams const& s, Policy const& pol,
                   std::vector<CCRect> const& hazards,
                   std::vector<CCPoint>* outPath = nullptr) {
    double dx = 0, y = 0, vy = s.vy0, flip = s.flip0, accel = 0;
    bool grounded = s.grounded0;
    float w = s.start.size.width, h = s.start.size.height;
    double v16fly = s.mini ? 0.85 : 1.0;

    auto clampFly = [&](double lo, double hi) {   // gravity-relative clamp
        double u = vy * flip;
        if (u < lo) u = lo;
        if (u > hi) u = hi;
        vy = u * flip;
    };
    auto clampFall = [&]() { if (vy * flip < -kFallClamp) vy = -kFallClamp * flip; };

    for (int i = 0; i < s.steps; i++) {
        bool click = false, held = s.held0;
        switch (pol.kind) {
            case 1: click = (i == pol.a); break;
            case 2:
                if (i >= pol.a) held = !s.held0;
                if (pol.b > 0 && i >= pol.a + pol.b) held = s.held0;
                break;
            case 3: held = (i >= pol.a && i < pol.a + pol.b); click = (i == pol.a); break;
            case 4: click = (i >= pol.a && pol.b > 0 && (i - pol.a) % pol.b == 0); break;
            default: break;
        }

        switch (s.mode) {
        case Mode::Cube:
            if (click && grounded) { vy = flip * s.yStart * s.jumpScale; grounded = false; }
            else if (!grounded)    { vy -= s.gBase * kDt * flip; clampFall(); }
            break;
        case Mode::Ball:
            if (click && grounded) {
                vy = flip * s.yStart * s.jumpScale;
                flip = -flip;
                vy *= 0.6;
                grounded = false;
            } else if (!grounded) { vy -= s.gAlt * kDt * flip * 0.6; clampFall(); }
            break;
        case Mode::Robot:
            if ((click || held) && grounded) {
                vy = flip * s.yStart * 0.5 * s.jumpScale; grounded = false; accel = 0;
            } else if (!grounded) {
                if (held && accel < 1.5) accel += kDt / 10.0;      // gravity cancelled while boosting
                else { vy -= s.gBase * kDt * flip * 0.9; clampFall(); }
            }
            break;
        case Mode::Spider:
            if (click && grounded) {                // teleports to the opposite surface
                flip = -flip;
                y = flip > 0 ? s.surfDown : s.surfUp;
                vy = 0; grounded = true;
            } else if (!grounded) { vy -= s.gAlt * kDt * flip * 0.6; clampFall(); }
            break;
        case Mode::Ship: {
            double dv = held ? (s.gAlt * kDt * 0.4 / v16fly)
                             : (-1.2 * s.gAlt * kDt * 0.4 / v16fly);
            vy += dv * flip;
            clampFly(-6.4 / v16fly, 8.0 / v16fly);
            break;
        }
        case Mode::Ufo: {
            if (click) {
                double v = (s.mini ? 8.0 : 7.0) * v16fly;
                if (v > vy * flip) vy = flip * v;
            }
            vy -= s.gAlt * kDt * flip * 1.2 * 0.5 / v16fly;
            clampFly(-6.4 / v16fly, 8.0 / v16fly);
            break;
        }
        case Mode::Wave:
            vy = flip * s.waveV * (held ? 1.0 : -1.0);
            break;
        case Mode::Swing:
            if (click) { vy *= 0.8; flip = -flip; }
            vy -= s.gAlt * kDt * flip * (s.mini ? 0.6 : 0.4);
            clampFly(-8.0, 8.0);
            break;
        }

        if (!grounded) y += vy * kDt;
        dx += s.vx * kStepSeconds;

        // surfaces: land on them (ground vehicles) or slide along them (flying/wave)
        double floorY = groundMode(s.mode) ? s.surfDown : s.floorRel;
        double ceilY  = groundMode(s.mode) ? s.surfUp   : s.ceilRel;
        if (y <= floorY) {
            y = floorY; if (vy < 0) vy = 0;
            if (groundMode(s.mode) && flip > 0) grounded = true;
        }
        if (y >= ceilY) {
            y = ceilY; if (vy > 0) vy = 0;
            if (groundMode(s.mode) && flip < 0) grounded = true;
        }

        CCRect r(s.start.origin.x + (float)dx, s.start.origin.y + (float)y, w, h);
        if (outPath && i % 6 == 0) outPath->push_back({r.getMidX(), r.getMidY()});

        for (auto const& hz : hazards) {
            CCRect t(hz.origin.x - s.margin, hz.origin.y - s.margin,
                     hz.size.width + 2 * s.margin, hz.size.height + 2 * s.margin);
            if (r.intersectsRect(t)) return i;
        }
    }
    return s.steps;
}

// Candidate input plans, ordered by how soon they act (earliest first).
static std::vector<Policy> candidates(Mode m, int steps) {
    std::vector<Policy> v;
    if (m == Mode::Robot) {
        for (int a : {0, 2, 4, 8, 14, 22, 32})
            for (int b : {10, 20, 40, 60}) v.push_back({3, a, b});
    } else if (m == Mode::Ship || m == Mode::Wave) {
        for (int a : {0, 3, 6, 10, 15, 22, 32, 45, 60}) {
            v.push_back({2, a, 0});
            for (int b : {8, 16, 32}) v.push_back({2, a, b});
        }
    } else {
        int lim = std::min(steps / 2, 60);
        for (int a = 0; a <= lim; a += (a < 12 ? 1 : 3)) v.push_back({1, a, 0});
        if (m == Mode::Ufo || m == Mode::Swing) {
            for (int a : {0, 6, 12, 20})
                for (int b : {10, 16, 24, 36}) v.push_back({4, a, b});
        }
    }
    return v;
}

// ================================================================= state
struct PFState {
    bool  enabled      = true;    // drive the cube with the look-ahead
    bool  record       = true;    // record inputs for .gdr2
    bool  autoSave     = true;    // save when the level is completed
    bool  overlay      = true;
    int   horizon      = 130;     // steps (240 Hz) to look ahead
    float margin       = 2.f;
    int   cooldown     = 24;      // steps between jumps (cube/ball/robot/spider)

    // live info
    bool  inLevel      = false;
    float nextHazard   = -1.f;
    bool  jumpSafe     = true;
    int   jumps        = 0;
    bool  dirty        = false;   // run had a practice respawn -> may not replay cleanly
    char const* modeName = "-";
    std::string status = "Ready.";
};
static PFState g;

struct Runtime {
    std::vector<GameObject*> hazards;       // sorted by x
    std::vector<std::pair<uint64_t, bool>> inputs; // (frame, down)
    bool   held = false;                  // real P1 input state
    int    releaseAt = -1;
    int    lastJump = -100000;
    int    lastProg = -1;
    double floorBottom = 90.0;            // learned level bounds (game coords)
    double ceilTop     = 390.0;
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
    rep.author      = std::string(GJAccountManager::get()->m_username.c_str());
    rep.description = "Generated by Pathfinder";
    rep.gameVersion = 22081;
    rep.framerate   = 240.0;
    rep.seed        = 0;
    rep.coins       = 0;
    rep.ldm         = false;
    rep.platformer  = pl->m_level->isPlatformer();
    rep.levelInfo.id   = pl->m_level->m_levelID.value();
    rep.levelInfo.name = std::string(pl->m_level->m_levelName.c_str());

    uint64_t last = 0;
    for (auto const& [frame, down] : rt.inputs) {
        rep.inputs.push_back(gdr::Input<>(frame, 1, false, down));
        last = frame;
    }
    rep.duration = (float)(last / 240.0);

    auto bytes = unwrapBytes(rep.exportData());
    if (!bytes) { g.status = "Export failed (GDR could not serialize the replay)."; return false; }

    std::string name = std::string(pl->m_level->m_levelName.c_str());
    for (auto& c : name) if (!std::isalnum((unsigned char)c)) c = '_';
    auto stamp = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    auto path = replayDir() / fmt::format("{}_{}_{}.gdr2", name, (int)pl->m_level->m_levelID.value(), stamp);

    auto res = geode::utils::file::writeBinary(path, *bytes);
    if (!res) { g.status = "Could not write file: " + path.string(); return false; }

    g.status = "Saved " + path.filename().string() + (g.dirty ? "  (had practice respawns)" : "");
    log::info("Pathfinder saved {} ({} inputs)", path.string(), rep.inputs.size());
    return true;
}

// ================================================================= UI
static void drawUI() {
    ImGui::SetNextWindowSize({400, 500}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos({40, 40}, ImGuiCond_FirstUseEver);
    ImGui::Begin("Pathfinder");

    ImGui::Checkbox("Drive with look-ahead", &g.enabled);
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
        ImGui::Text("Mode: %s", g.modeName);
        ImGui::Text("Current input is: %s", g.jumpSafe ? "safe" : "unsafe");
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
    ImGui::TextWrapped("All gamemodes. Cube is the best tested; other modes are approximate. Hazards only: blocks/orbs/pads/portals are not modelled.");
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
static void learnBounds(PlayerObject* p, Mode m) {
    // Only trust surface contact in modes that live between floor and ceiling.
    if (m == Mode::Cube || m == Mode::Robot || m == Mode::Spider) return;
    if (!p->m_isOnGround) return;
    auto r = p->getObjectRect();
    if (p->m_isUpsideDown) rt.ceilTop = r.getMaxY();
    else                   rt.floorBottom = r.getMinY();
}

static SimParams buildParams(PlayerObject* p, Mode m) {
    auto prof = profileFor(p->m_playerSpeed);
    SimParams s;
    s.start     = p->getObjectRect();
    s.mode      = m;
    s.mini      = p->m_vehicleSize != 1.f;
    s.vx        = p->m_playerSpeed * prof.speedMult * 60.0;
    s.yStart    = prof.yStart;
    s.jumpScale = s.mini ? 0.8 : 1.0;
    s.gBase     = prof.gravity * p->m_gravityMod;
    s.gAlt      = kFlyGravity * p->m_gravityMod;
    s.waveV     = p->m_playerSpeed * prof.speedMult;
    s.flip0     = p->m_isUpsideDown ? -1.0 : 1.0;
    s.vy0       = p->m_yVelocity;
    s.grounded0 = p->m_isOnGround;
    s.held0     = rt.held;
    s.steps     = g.horizon;
    s.margin    = g.margin;

    double bottom = s.start.origin.y, top = bottom + s.start.size.height;
    s.floorRel = std::min(0.0, rt.floorBottom - bottom);
    s.ceilRel  = std::max(0.0, rt.ceilTop - top);
    // ground vehicles land on whatever they are standing on right now
    s.surfDown = (s.grounded0 && s.flip0 > 0) ? 0.0 : s.floorRel;
    s.surfUp   = (s.grounded0 && s.flip0 < 0) ? 0.0 : s.ceilRel;
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
        if (std::abs(h->getPositionY() - p->getPositionY()) > 350.f) continue;
        out.push_back(h->getObjectRect());
        if (g.nextHazard < 0.f) g.nextHazard = dx;
        if (draw && rt.draw) rt.draw->drawDot(h->getPosition(), 7.f, {1.f, 0.2f, 0.2f, 0.7f});
    }
    return out;
}

static void act(PlayLayer* pl, Policy const& c, int prog) {
    switch (c.kind) {
        case 1: case 4:
            pl->handleButton(true, 1, true);
            rt.releaseAt = prog + 1;
            rt.lastJump = prog;
            break;
        case 2:
            pl->handleButton(!rt.held, 1, true);   // flip hold state
            break;
        case 3:
            pl->handleButton(true, 1, true);
            rt.releaseAt = prog + std::max(1, c.b);
            rt.lastJump = prog;
            break;
        default: return;
    }
    g.jumps++;
}

// Runs once per 240 Hz physics step, right before the game processes input.
static void tick(PlayLayer* pl) {
    int prog = progress(pl);
    if (prog == rt.lastProg) return;            // one decision per physics step
    rt.lastProg = prog;

    // release a tap/pulse when it is due
    if (rt.releaseAt >= 0 && prog >= rt.releaseAt) {
        pl->handleButton(false, 1, true);
        rt.releaseAt = -1;
    }

    auto p = pl->m_player1;
    if (!p || p->m_isDead) return;
    Mode m = modeOf(p);
    learnBounds(p, m);

    if (!g.enabled) return;
    if (rt.releaseAt >= 0) return;               // a tap/pulse is in progress
    if (groundMode(m) && !p->m_isOnGround) return;
    int cd = (m == Mode::Ufo || m == Mode::Swing) ? 6 : g.cooldown;
    if (!holdMode(m) || m == Mode::Robot) {
        if (prog - rt.lastJump < cd) return;
    }

    SimParams s = buildParams(p, m);
    auto near = nearHazards(p, s, false);
    if (near.empty()) return;

    Policy none{0, 0, 0};
    int base = survive(s, none, near);
    if (base >= s.steps) return;                 // current input survives

    Policy best = none;
    int bestSurv = base;
    for (auto const& c : candidates(m, s.steps)) {
        int r = survive(s, c, near);
        if (r >= s.steps) { best = c; break; }   // earliest fully safe plan
        if (r > bestSurv) { bestSurv = r; best = c; }
    }
    if (best.kind == 0 || best.a != 0) return;   // nothing to do yet / wait
    act(pl, best, prog);
}

// ================================================================= hooks
class $modify(PFBaseGameLayer, GJBaseGameLayer) {
    void processCommands(float dt, bool isHalfTick, bool isLastTick) {
        auto pl = PlayLayer::get();
        if (pl && static_cast<GJBaseGameLayer*>(pl) == this) tick(pl);
        GJBaseGameLayer::processCommands(dt, isHalfTick, isLastTick);
    }

    // Same place every bot records from, so frames match what Eclipse replays.
    void handleButton(bool down, int button, bool isPlayer1) {
        GJBaseGameLayer::handleButton(down, button, isPlayer1);

        auto pl = PlayLayer::get();
        if (!pl || static_cast<GJBaseGameLayer*>(pl) != this) return;
        if (button != 1 || !isPlayer1) return;   // this mod only drives / records P1
        rt.held = down;                          // real input state, used by the planner

        if (!g.record || !m_player1 || m_player1->m_isDead) return;
        if (rt.inputs.empty() && !down) return;
        if (!rt.inputs.empty() && rt.inputs.back().second == down) return;
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
        rt.lastProg = -1;
        rt.floorBottom = 90.0;
        rt.ceilTop = 390.0;

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
        rt.lastProg = -1;
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
        if (!p || p->m_isDead) return;

        Mode m = modeOf(p);
        g.modeName = modeName(m);
        SimParams s = buildParams(p, m);
        auto near = nearHazards(p, s, g.overlay);
        Policy none{0, 0, 0};
        g.jumpSafe = survive(s, none, near) >= s.steps;

        if (g.overlay && rt.draw) {
            std::vector<CCPoint> path;
            survive(s, none, near, &path);
            ccColor4F col = g.jumpSafe ? ccColor4F{0.2f, 1.f, 0.4f, 0.8f}
                                       : ccColor4F{1.f, 0.6f, 0.1f, 0.8f};
            for (auto const& pt : path) rt.draw->drawDot(pt, 2.5f, col);
        }
    }
};
