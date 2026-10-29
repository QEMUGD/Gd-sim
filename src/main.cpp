// Pathfinder - Geode mod for Geometry Dash 2.2081 (Windows + Android)
// Author: qemugd  https://github.com/qemugd/gd-sim
//
// Press the cube button on a level page, pick settings, press OK. The mod maps
// the level (hazards + solid blocks), plays it with a physics look-ahead, logs
// every failed attempt, re-maps with extra caution around the failure and tries
// again until it passes. Every input is recorded on its exact physics frame and
// exported as a .gdr2 replay (Eclipse Menu and other GDR2 bots).
//
// Credits
//   * Physics model: camila314's GD physics decompilations (camila314/gdp,
//     2.2 branch, used with permission). Constants and formulas below come from
//       - PlayerObject::updateTimeMod  (yStart / gravity / speed per speed portal)
//       - PlayerObject::updateJump     (jump rules for every vehicle, mini scaling,
//                                       gravity formula, fall clamps)
//       - GJBaseGameLayer::update      (step count = delta * 240, physicsDelta)
//       - PlayLayer::destroyPlayer     (death handling)
//   * Replay format: maxnut/GDReplayFormat (gdr2)
//   * ImGui for Cocos2d: matcool/gd-imgui-cocos
//   * Built on Geode.

#include <Geode/Geode.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <Geode/modify/MenuLayer.hpp>
#include <Geode/modify/PauseLayer.hpp>
#include <Geode/modify/LevelInfoLayer.hpp>
#include <imgui-cocos.hpp>
#include <gdr/gdr.hpp>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <deque>

using namespace geode::prelude;

// ==== SIM BEGIN
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
    double floorRel, ceilRel;  // level bounds, relative to start y
    int    lockSteps;   // CPS lock: min 240 Hz steps between presses (0 = off)
    int    sinceLast;   // 240 Hz steps since the last press
    int    steps;
    float  margin;
};

// kind: 0 keep current input, 1 click at a, 2 toggle hold at a (and back after b),
//       3 hold from a for b steps, 4 click at a then every b steps
struct Policy { int kind, a, b; };

// Simulates the vehicle in 240 Hz steps and returns how many steps it survives
// (== s.steps means no hazard / wall was touched). `hz` and `sol` must be sorted
// by minX. The CPS lock is enforced here so plans never rely on a blocked click.
static int survive(SimParams const& s, Policy const& pol,
                   std::vector<CCRect> const& hz, std::vector<CCRect> const& sol,
                   std::vector<CCPoint>* outPath = nullptr) {
    double dx = 0, y = 0, vy = s.vy0, flip = s.flip0, accel = 0;
    bool grounded = s.grounded0, prevHeld = s.held0;
    int lastPress = -s.sinceLast;
    float w = s.start.size.width, h = s.start.size.height;
    double v16fly = s.mini ? 0.85 : 1.0;
    size_t hLo = 0, sLo = 0;

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
        // CPS lock
        bool press = click || (held && !prevHeld);
        if (press) {
            if (s.lockSteps <= 0 || i - lastPress >= s.lockSteps) lastPress = i;
            else { click = false; if (held && !prevHeld) held = false; }
        }
        prevHeld = held;

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
                y = flip > 0 ? s.floorRel : s.ceilRel;
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

        // level floor / ceiling: land on them (ground vehicles) or slide along them
        bool supported = false;
        if (y <= s.floorRel) {
            y = s.floorRel; if (vy < 0) vy = 0;
            if (flip > 0) { supported = true; if (groundMode(s.mode)) grounded = true; }
        }
        if (y >= s.ceilRel) {
            y = s.ceilRel; if (vy > 0) vy = 0;
            if (flip < 0) { supported = true; if (groundMode(s.mode)) grounded = true; }
        }

        // solid blocks: land on top, bump underneath, die on the sides
        double bx = s.start.origin.x + dx;
        double rMinX = bx, rMaxX = bx + w;
        while (sLo < sol.size() && sol[sLo].getMaxX() < rMinX - 2) sLo++;
        for (size_t j = sLo; j < sol.size() && sol[j].getMinX() <= rMaxX + 2; ++j) {
            auto const& b = sol[j];
            double rMinY = s.start.origin.y + y, rMaxY = rMinY + h;
            bool hOver = rMaxX > b.getMinX() + 0.5 && rMinX < b.getMaxX() - 0.5;
            if (!hOver) continue;
            if (grounded) {   // resting contact keeps us supported
                if (flip > 0 && std::abs(b.getMaxY() - rMinY) <= 2.0) supported = true;
                if (flip < 0 && std::abs(b.getMinY() - rMaxY) <= 2.0) supported = true;
            }
            bool vOver = rMaxY > b.getMinY() + 1.0 && rMinY < b.getMaxY() - 1.0;
            if (!vOver) continue;
            if (s.mode == Mode::Wave) return i;                 // wave dies on any block
            double tol = std::max(4.0, std::abs(vy) * kDt + 2.0);
            double feetPen = flip > 0 ? b.getMaxY() - rMinY : rMaxY - b.getMinY();
            double headPen = flip > 0 ? rMaxY - b.getMinY() : b.getMaxY() - rMinY;
            if (feetPen <= tol && vy * flip <= 0.0) {
                y += flip * feetPen; vy = 0; supported = true;
                if (groundMode(s.mode)) grounded = true;
            } else if (headPen <= tol && vy * flip >= 0.0) {
                y -= flip * headPen; if (vy * flip > 0) vy = 0;
            } else {
                return i;                                        // side hit
            }
        }
        if (grounded && !supported) { grounded = false; vy = 0; } // ran off an edge

        CCRect r((float)bx, (float)(s.start.origin.y + y), w, h);
        if (outPath && i % 6 == 0) outPath->push_back({r.getMidX(), r.getMidY()});

        // hazards
        while (hLo < hz.size() && hz[hLo].getMaxX() < rMinX - s.margin - 2) hLo++;
        for (size_t j = hLo; j < hz.size() && hz[j].getMinX() <= rMaxX + s.margin + 2; ++j) {
            auto const& hzr = hz[j];
            CCRect t(hzr.origin.x - s.margin, hzr.origin.y - s.margin,
                     hzr.size.width + 2 * s.margin, hzr.size.height + 2 * s.margin);
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
// ==== SIM END

// ================================================================= state
struct PFState {
    // ---- settings (saved)
    bool  enabled      = false;   // manual drive (outside a pathfinder session)
    bool  record       = true;
    bool  autoSave     = true;
    bool  overlay      = true;
    int   horizon      = 130;     // look-ahead, in 240 Hz steps
    float margin       = 2.f;
    int   cooldown     = 24;      // steps between presses (cube/ball/robot/spider)
    float speed        = 10.f;    // game speed during a pathfinder session
    int   tps          = 240;     // physics steps per second (240 = vanilla)
    bool  cpsLock      = true;    // limit presses per second
    int   cps          = 16;
    int   maxAttempts  = 300;

    // ---- ui
    bool  showMain     = false;
    bool  showSetup    = false;
    Ref<GJGameLevel> setupLevel;

    // ---- live
    bool  inLevel      = false;
    float nextHazard   = -1.f;
    bool  inputSafe    = true;
    int   jumps        = 0;
    bool  dirty        = false;
    float percent      = 0.f;
    char const* modeName = "-";
    std::string status = "Ready.";
};
static PFState g;

enum class Phase { Idle, Mapping, Botting, Passed, GaveUp };
struct Zone { float x0, x1; double extra; int strategy; int fails; };
struct Session {
    bool  armed = false, active = false;
    Phase phase = Phase::Idle;
    int   attempt = 0;
    float best = 0.f;
    std::vector<Zone> zones;
};
static Session sess;

struct LevelMap {
    std::vector<CCRect> hz, sol;   // sorted by minX
    float hzMaxW = 0, solMaxW = 0, length = 0;
};
static LevelMap lmap;

struct Runtime {
    std::vector<std::pair<uint64_t, bool>> inputs; // (frame, down)
    bool   held = false;                  // real P1 input state
    int    releaseAt = -1;
    int    lastJump = -100000;
    int    lastPress = -100000;
    int    lastProg = -1;
    double floorBottom = 90.0;            // level bounds (game coords)
    double ceilTop     = 390.0;
    double accum = 0.0;                   // TPS bypass time accumulator
    CCDrawNode* draw = nullptr;
    CCLabelBMFont* label = nullptr;
};
static Runtime rt;

static int progress(GJBaseGameLayer* l) { return (int)l->m_gameState.m_currentProgress; }

// ================================================================= log
static std::deque<std::string> logLines;

static void plog(std::string const& line) {
    logLines.push_back(line);
    while (logLines.size() > 400) logLines.pop_front();
    log::info("Pathfinder: {}", line);
    std::ofstream f(Mod::get()->getSaveDir() / "pathfinder_log.txt", std::ios::app);
    if (f) f << line << "\n";
}

// ================================================================= settings persistence
static void saveSettings() {
    auto m = Mod::get();
    m->setSavedValue("record", g.record);   m->setSavedValue("autosave", g.autoSave);
    m->setSavedValue("overlay", g.overlay); m->setSavedValue("horizon", g.horizon);
    m->setSavedValue("margin", g.margin);   m->setSavedValue("cooldown", g.cooldown);
    m->setSavedValue("speed", g.speed);     m->setSavedValue("tps", g.tps);
    m->setSavedValue("cpslock", g.cpsLock); m->setSavedValue("cps", g.cps);
    m->setSavedValue("maxattempts", g.maxAttempts);
}
static void loadSettings() {
    auto m = Mod::get();
    g.record = m->getSavedValue<bool>("record", true);
    g.autoSave = m->getSavedValue<bool>("autosave", true);
    g.overlay = m->getSavedValue<bool>("overlay", true);
    g.horizon = m->getSavedValue<int>("horizon", 130);
    g.margin = m->getSavedValue<float>("margin", 2.f);
    g.cooldown = m->getSavedValue<int>("cooldown", 24);
    g.speed = m->getSavedValue<float>("speed", 10.f);
    g.tps = m->getSavedValue<int>("tps", 240);
    g.cpsLock = m->getSavedValue<bool>("cpslock", true);
    g.cps = m->getSavedValue<int>("cps", 16);
    g.maxAttempts = m->getSavedValue<int>("maxattempts", 300);
    g.tps = std::clamp(g.tps, 240, 5000);
    g.speed = std::clamp(g.speed, 1.f, 10.f);
}

static void setTimeScale(float v) {
    if (auto d = CCDirector::get()) {
        if (auto sch = d->getScheduler()) sch->setTimeScale(v);
    }
}

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
    rep.framerate   = (double)g.tps;           // physics steps per second the run was played at
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
    rep.duration = (float)(last / (double)g.tps);

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
    plog(fmt::format("Saved replay {} ({} inputs, {} TPS)", path.filename().string(), rep.inputs.size(), g.tps));
    return true;
}

// ================================================================= level mapping
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

// Scans every object once and stores static hitboxes of hazards and solid blocks.
static void mapLevel(PlayLayer* pl) {
    lmap.hz.clear(); lmap.sol.clear();
    lmap.hzMaxW = lmap.solMaxW = lmap.length = 0.f;
    for (auto obj : CCArrayExt<GameObject*>(pl->m_objects)) {
        if (!obj) continue;
        bool hazard = obj->m_objectType == GameObjectType::Hazard;
        bool solid  = obj->m_objectType == GameObjectType::Solid;
        if (!hazard && !solid) continue;
        CCRect r = obj->getObjectRect();
        if (r.size.width <= 0.f || r.size.height <= 0.f) continue;
        if (hazard) { lmap.hz.push_back(r);  lmap.hzMaxW  = std::max(lmap.hzMaxW,  r.size.width); }
        else        { lmap.sol.push_back(r); lmap.solMaxW = std::max(lmap.solMaxW, r.size.width); }
        lmap.length = std::max(lmap.length, r.getMaxX());
    }
    auto byX = [](CCRect const& a, CCRect const& b) { return a.getMinX() < b.getMinX(); };
    std::sort(lmap.hz.begin(),  lmap.hz.end(),  byX);
    std::sort(lmap.sol.begin(), lmap.sol.end(), byX);
}

// copies the objects in [x0, x1] near height py out of a sorted list
static void sliceMap(std::vector<CCRect> const& all, float maxW, float x0, float x1,
                     float py, std::vector<CCRect>& out) {
    out.clear();
    auto it = std::lower_bound(all.begin(), all.end(), x0 - maxW,
        [](CCRect const& r, float v) { return r.getMinX() < v; });
    for (; it != all.end() && it->getMinX() <= x1; ++it) {
        if (std::abs(it->getMidY() - py) <= 380.f) out.push_back(*it);
    }
}

// ================================================================= decision logic
static void learnBounds(PlayerObject* p, Mode m) {
    // Only trust surface contact in modes that live between floor and ceiling.
    if (m == Mode::Cube || m == Mode::Robot || m == Mode::Spider) return;
    if (!p->m_isOnGround) return;
    auto r = p->getObjectRect();
    if (p->m_isUpsideDown) rt.ceilTop = r.getMaxY();
    else                   rt.floorBottom = r.getMinY();
}

// extra caution learned from failed attempts near x
static Zone const* zoneAt(float x) {
    Zone const* best = nullptr;
    for (auto const& z : sess.zones) {
        if (x >= z.x0 && x <= z.x1 && (!best || z.extra > best->extra)) best = &z;
    }
    return best;
}

static SimParams buildParams(PlayerObject* p, Mode m, int prog) {
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

    auto z = zoneAt(p->getPositionX());
    s.margin = g.margin + (z ? (float)z->extra : 0.f);

    s.lockSteps = (sess.active || g.enabled) && g.cpsLock && g.cps > 0
                    ? (int)std::lround(240.0 / g.cps) : 0;
    s.sinceLast = (int)std::min<double>(100000.0, (prog - rt.lastPress) * 240.0 / std::max(1, g.tps));

    double bottom = s.start.origin.y, top = bottom + s.start.size.height;
    s.floorRel = std::min(0.0, rt.floorBottom - bottom);
    s.ceilRel  = std::max(0.0, rt.ceilTop - top);
    return s;
}

static void act(PlayLayer* pl, Policy const& c, int prog) {
    int dec = std::max(1, g.tps / 240);
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
            rt.releaseAt = prog + std::max(1, c.b) * dec;
            rt.lastJump = prog;
            break;
        default: return;
    }
    g.jumps++;
}

// Runs once per physics step, right before the game processes input.
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
    if (!(g.enabled || sess.active)) return;

    int dec = std::max(1, g.tps / 240);          // at high TPS decide ~240 times per second
    if (prog % dec != 0) return;

    Mode m = modeOf(p);
    learnBounds(p, m);

    if (rt.releaseAt >= 0) return;               // a tap/pulse is in progress
    if (groundMode(m) && !p->m_isOnGround) return;
    int cd = ((m == Mode::Ufo || m == Mode::Swing) ? 6 : g.cooldown) * dec;
    if (!holdMode(m) || m == Mode::Robot) {
        if (prog - rt.lastJump < cd) return;
    }

    SimParams s = buildParams(p, m, prog);
    float px = p->getPositionX();
    double reach = s.vx * s.steps * kStepSeconds + 80.0;
    static std::vector<CCRect> hzW, solW;
    sliceMap(lmap.hz,  lmap.hzMaxW,  px - 40.f, (float)(px + reach), p->getPositionY(), hzW);
    sliceMap(lmap.sol, lmap.solMaxW, px - 40.f, (float)(px + reach), p->getPositionY(), solW);
    if (hzW.empty() && solW.empty()) return;

    Policy none{0, 0, 0};
    int base = survive(s, none, hzW, solW);
    if (base >= s.steps) return;                 // current input survives

    auto z = zoneAt(px);
    int strategy = z ? z->strategy : 0;          // 0 earliest, 1 latest, 2 middle safe plan

    std::vector<Policy> safe;
    Policy fallback = none;
    int bestSurv = base;
    for (auto const& c : candidates(m, s.steps)) {
        int r = survive(s, c, hzW, solW);
        if (r >= s.steps) { safe.push_back(c); if (strategy == 0) break; }
        else if (r > bestSurv) { bestSurv = r; fallback = c; }
    }
    Policy pick = fallback;
    if (!safe.empty()) {
        pick = strategy == 1 ? safe.back()
             : strategy == 2 ? safe[safe.size() / 2]
             : safe.front();
    }
    if (pick.kind == 0 || pick.a != 0) return;   // nothing to do yet / wait

    // never fire a press the CPS lock would block
    if (s.lockSteps > 0 && s.sinceLast < s.lockSteps && pick.kind != 2) return;
    act(pl, pick, prog);
}

// ================================================================= pathfinder session
static void beginSession(PlayLayer* pl) {
    sess.armed = false;
    sess.active = true;
    sess.phase = Phase::Mapping;
    sess.attempt = 0;
    sess.best = 0.f;
    sess.zones.clear();
    plog(fmt::format("=== Pathfinder session: {} (ID {}) | {}x speed, {} TPS, CPS lock {} ===",
        std::string(pl->m_level->m_levelName.c_str()), (int)pl->m_level->m_levelID.value(),
        g.speed, g.tps, g.cpsLock ? std::to_string(g.cps) : std::string("off")));
    mapLevel(pl);
    plog(fmt::format("Mapped level: {} hazards, {} solid blocks, length {:.0f} units.",
        lmap.hz.size(), lmap.sol.size(), lmap.length));
    sess.phase = Phase::Botting;
    setTimeScale(g.speed);
}

static void endSession(char const* why, Phase result) {
    if (!sess.active) return;
    sess.active = false;
    sess.phase = result;
    setTimeScale(1.f);
    plog(why);
}

static void onFailure(PlayLayer* pl) {
    auto p = pl->m_player1;
    float x = p ? p->getPositionX() : 0.f;
    float pct = pl->getCurrentPercent();
    sess.attempt++;
    sess.best = std::max(sess.best, pct);

    // widen the caution around the place where decisions were made
    Zone* hit = nullptr;
    for (auto& z : sess.zones) if (x >= z.x0 && x <= z.x1) { hit = &z; break; }
    if (hit) {
        hit->fails++;
        hit->extra = std::min(6.0, hit->extra + 1.0);
        hit->strategy = (hit->strategy + 1) % 3;
    } else {
        sess.zones.push_back({x - 220.f, x + 10.f, 1.0, 0, 1});
        hit = &sess.zones.back();
    }
    plog(fmt::format("[attempt {}] FAILED at {:.1f}% (x={:.0f}, {}) | zone +{:.0f} margin, strategy {}, fails here {}",
        sess.attempt, pct, x, p ? modeName(modeOf(p)) : "?", hit->extra, hit->strategy, hit->fails));

    mapLevel(pl);   // re-map before the next try
    plog(fmt::format("Re-mapped ({} hazards, {} blocks). Trying again...", lmap.hz.size(), lmap.sol.size()));

    if (sess.attempt >= g.maxAttempts) {
        endSession(fmt::format("Gave up after {} attempts (best {:.1f}%).", sess.attempt, sess.best).c_str(),
                   Phase::GaveUp);
    }
}

static void startPathfind() {
    if (!g.setupLevel) return;
    saveSettings();
    sess.armed = true;
    g.showSetup = false;
    g.showMain = false;
    auto scene = PlayLayer::scene(g.setupLevel, false, false);
    CCDirector::get()->replaceScene(CCTransitionFade::create(0.5f, scene));
}

// ================================================================= UI
static void applyTheme() {
    auto& st = ImGui::GetStyle();
    st.WindowRounding = 14.f;  st.ChildRounding = 10.f; st.FrameRounding = 9.f;
    st.PopupRounding = 10.f;   st.GrabRounding = 9.f;   st.TabRounding = 9.f;
    st.ScrollbarRounding = 9.f;
    st.WindowPadding = {14.f, 12.f}; st.FramePadding = {10.f, 7.f}; st.ItemSpacing = {10.f, 9.f};
    st.WindowBorderSize = 0.f; st.FrameBorderSize = 0.f;
    auto C = [](float r, float g_, float b, float a = 1.f) { return ImVec4(r, g_, b, a); };
    auto* c = st.Colors;
    c[ImGuiCol_WindowBg]       = C(0.07f, 0.07f, 0.11f, 0.97f);
    c[ImGuiCol_ChildBg]        = C(0.05f, 0.05f, 0.09f, 1.f);
    c[ImGuiCol_PopupBg]        = C(0.09f, 0.09f, 0.14f, 0.98f);
    c[ImGuiCol_Text]           = C(0.93f, 0.94f, 1.00f);
    c[ImGuiCol_TextDisabled]   = C(0.50f, 0.52f, 0.62f);
    c[ImGuiCol_FrameBg]        = C(0.13f, 0.13f, 0.21f);
    c[ImGuiCol_FrameBgHovered] = C(0.18f, 0.18f, 0.29f);
    c[ImGuiCol_FrameBgActive]  = C(0.22f, 0.22f, 0.36f);
    c[ImGuiCol_Button]         = C(0.27f, 0.40f, 0.88f);
    c[ImGuiCol_ButtonHovered]  = C(0.36f, 0.50f, 0.98f);
    c[ImGuiCol_ButtonActive]   = C(0.45f, 0.30f, 0.95f);
    c[ImGuiCol_SliderGrab]     = C(0.45f, 0.62f, 1.00f);
    c[ImGuiCol_SliderGrabActive] = C(0.70f, 0.45f, 1.00f);
    c[ImGuiCol_CheckMark]      = C(0.55f, 0.85f, 1.00f);
    c[ImGuiCol_Header]         = C(0.20f, 0.24f, 0.45f);
    c[ImGuiCol_HeaderHovered]  = C(0.28f, 0.34f, 0.62f);
    c[ImGuiCol_HeaderActive]   = C(0.36f, 0.30f, 0.70f);
    c[ImGuiCol_Tab]            = C(0.13f, 0.14f, 0.26f);
    c[ImGuiCol_TabHovered]     = C(0.36f, 0.50f, 0.98f);
    c[ImGuiCol_TabActive]      = C(0.30f, 0.42f, 0.90f);
    c[ImGuiCol_PlotHistogram]  = C(0.45f, 0.62f, 1.00f);
    c[ImGuiCol_Separator]      = C(0.20f, 0.22f, 0.34f);
    c[ImGuiCol_TitleBg]        = C(0.07f, 0.07f, 0.11f);
    c[ImGuiCol_TitleBgActive]  = C(0.10f, 0.11f, 0.20f);
}

// the Geometry Dash cube, drawn with the draw list
static void drawCube(ImDrawList* dl, ImVec2 p, float sz) {
    dl->AddRectFilled(p, {p.x + sz, p.y + sz}, IM_COL32(0, 0, 0, 255), sz * 0.06f);
    float b = sz * 0.07f;
    dl->AddRectFilled({p.x + b, p.y + b}, {p.x + sz - b, p.y + sz - b}, IM_COL32(120, 255, 70, 255), sz * 0.04f);
    float e = sz * 0.22f, ey = p.y + sz * 0.30f;
    dl->AddRectFilled({p.x + sz * 0.20f, ey}, {p.x + sz * 0.20f + e, ey + e}, IM_COL32(0, 0, 0, 255));
    dl->AddRectFilled({p.x + sz * 0.58f, ey}, {p.x + sz * 0.58f + e, ey + e}, IM_COL32(0, 0, 0, 255));
    dl->AddRectFilled({p.x + sz * 0.22f, p.y + sz * 0.68f}, {p.x + sz * 0.78f, p.y + sz * 0.82f}, IM_COL32(0, 0, 0, 255));
}

static void drawHeader(char const* title, char const* sub) {
    ImVec2 p = ImGui::GetCursorScreenPos();
    float w = ImGui::GetContentRegionAvail().x;
    float fs = ImGui::GetFontSize();
    float h = fs * 3.4f;
    auto* dl = ImGui::GetWindowDrawList();
    ImU32 a = IM_COL32(70, 120, 255, 255), b = IM_COL32(150, 80, 255, 255);
    dl->AddRectFilledMultiColor(p, {p.x + w, p.y + h}, a, b, b, a);
    dl->AddText(ImGui::GetFont(), fs * 1.55f, {p.x + 16.f, p.y + fs * 0.45f}, IM_COL32_WHITE, title);
    dl->AddText({p.x + 16.f, p.y + fs * 2.2f}, IM_COL32(230, 230, 255, 255), sub);
    drawCube(dl, {p.x + w - fs * 2.6f, p.y + fs * 0.45f}, fs * 2.4f);
    ImGui::Dummy({w, h});
}

static void drawSettings() {
    ImGui::SeparatorText("Speed");
    ImGui::SliderFloat("Game speed", &g.speed, 1.f, 10.f, "%.1fx");
    ImGui::SliderInt("TPS", &g.tps, 240, 5000, "%d", ImGuiSliderFlags_Logarithmic);
    ImGui::TextDisabled("240 = vanilla physics. Higher TPS is experimental and heavy.");
    ImGui::SeparatorText("Clicks");
    ImGui::Checkbox("CPS lock", &g.cpsLock);
    ImGui::BeginDisabled(!g.cpsLock);
    ImGui::SliderInt("Clicks per second", &g.cps, 1, 60);
    ImGui::EndDisabled();
    ImGui::SeparatorText("Pathfinding");
    ImGui::SliderInt("Max attempts", &g.maxAttempts, 1, 2000);
    ImGui::SliderInt("Look-ahead (steps)", &g.horizon, 60, 300);
    ImGui::SliderFloat("Safety margin", &g.margin, 0.f, 8.f, "%.1f");
    ImGui::SliderInt("Click cooldown", &g.cooldown, 8, 96);
    ImGui::SeparatorText("Output");
    ImGui::Checkbox("Record inputs (.gdr2)", &g.record);
    ImGui::Checkbox("Auto-save when passed", &g.autoSave);
    ImGui::Checkbox("Show hazards + predicted arc", &g.overlay);
}

static char const* phaseName(Phase p) {
    switch (p) {
        case Phase::Idle: return "Idle";       case Phase::Mapping: return "Mapping";
        case Phase::Botting: return "Botting"; case Phase::Passed: return "Passed";
        case Phase::GaveUp: return "Gave up";
    }
    return "?";
}

static void drawSetupWindow() {
    auto ds = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos({ds.x * 0.5f, ds.y * 0.5f}, ImGuiCond_Always, {0.5f, 0.5f});
    ImGui::SetNextWindowSize({std::min(460.f, ds.x * 0.92f), std::min(620.f, ds.y * 0.92f)}, ImGuiCond_Always);
    ImGui::Begin("Pathfinder setup", nullptr,
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoTitleBar);
    drawHeader("PATHFINDER", "Map the level, then bot it");

    if (g.setupLevel) {
        ImGui::Text("Level: %s", g.setupLevel->m_levelName.c_str());
        ImGui::TextDisabled("ID %d", (int)g.setupLevel->m_levelID.value());
    }
    ImGui::BeginChild("settings", {0, -ImGui::GetFrameHeightWithSpacing() * 2.2f}, false);
    drawSettings();
    ImGui::EndChild();

    bool platformer = g.setupLevel && g.setupLevel->isPlatformer();
    if (platformer) ImGui::TextColored({1.f, 0.6f, 0.3f, 1.f}, "Platformer levels are not supported.");
    float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    ImGui::BeginDisabled(platformer);
    if (ImGui::Button("OK  -  Start pathfinding", {half, 0})) startPathfind();
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel", {half, 0})) { g.showSetup = false; saveSettings(); }
    ImGui::End();
}

static void drawMainWindow() {
    ImGui::SetNextWindowSize({420, 560}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos({30, 30}, ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Pathfinder", &g.showMain, ImGuiWindowFlags_NoTitleBar)) { ImGui::End(); return; }
    drawHeader("PATHFINDER", "by qemugd  |  physics: camila314/gdp");

    if (ImGui::BeginTabBar("pf_tabs")) {
        if (ImGui::BeginTabItem("Status")) {
            ImGui::Text("Phase: %s", phaseName(sess.phase));
            if (g.inLevel) {
                ImGui::Text("Mode: %s", g.modeName);
                ImGui::Text("Attempt: %d   Best: %.1f%%", sess.attempt, sess.best);
                ImGui::ProgressBar(std::clamp(g.percent / 100.f, 0.f, 1.f), {-1, 0},
                    fmt::format("{:.1f}%", g.percent).c_str());
                if (g.nextHazard >= 0.f) ImGui::Text("Next hazard: %.0f units", g.nextHazard);
                else                     ImGui::Text("Next hazard: none in range");
                ImGui::Text("Current input is: %s", g.inputSafe ? "safe" : "unsafe");
                ImGui::Text("Inputs recorded: %d  (clicks: %d)", (int)rt.inputs.size(), g.jumps);
                if (g.dirty) ImGui::TextColored({1.f, 0.7f, 0.2f, 1.f}, "Run has practice respawns");
                ImGui::Checkbox("Manual drive (no session)", &g.enabled);
                if (ImGui::Button("Save .gdr2 now", {-1, 0})) {
                    if (auto pl = PlayLayer::get()) saveReplay(pl);
                }
                if (ImGui::Button("Clear inputs", {-1, 0})) { rt.inputs.clear(); rt.held = false; g.status = "Cleared."; }
            } else {
                ImGui::TextWrapped("Open a level page and press the cube button to pathfind it.");
            }
            ImGui::Separator();
            ImGui::TextWrapped("%s", g.status.c_str());
            ImGui::TextDisabled("Saves to: %s", replayDir().string().c_str());
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Settings")) {
            drawSettings();
            if (ImGui::Button("Save settings", {-1, 0})) { saveSettings(); g.status = "Settings saved."; }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Map")) {
            ImGui::Text("Hazards mapped: %d", (int)lmap.hz.size());
            ImGui::Text("Solid blocks mapped: %d", (int)lmap.sol.size());
            ImGui::Text("Level length: %.0f units", lmap.length);
            ImGui::Text("Floor %.0f  |  Ceiling %.0f", rt.floorBottom, rt.ceilTop);
            ImGui::SeparatorText("Caution zones (from failures)");
            if (sess.zones.empty()) ImGui::TextDisabled("none yet");
            for (auto const& z : sess.zones) {
                ImGui::BulletText("x %.0f-%.0f  +%.0f margin  strategy %d  fails %d",
                    z.x0, z.x1, z.extra, z.strategy, z.fails);
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Log")) {
            if (ImGui::Button("Copy")) {
                std::string all;
                for (auto const& l : logLines) all += l + "\n";
                ImGui::SetClipboardText(all.c_str());
            }
            ImGui::SameLine();
            if (ImGui::Button("Clear")) logLines.clear();
            ImGui::BeginChild("logchild", {0, 0}, true);
            for (auto const& l : logLines) ImGui::TextWrapped("%s", l.c_str());
            if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4.f) ImGui::SetScrollHereY(1.f);
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::TextDisabled("Format: maxnut/GDReplayFormat  |  UI: ImGui");
    if (ImGui::Button("Close", {-1, 0})) g.showMain = false;
    ImGui::End();
}

static void drawUI() {
    if (g.showSetup) drawSetupWindow();
    else if (g.showMain) drawMainWindow();
}

$on_mod(Loaded) {
    loadSettings();
    ImGuiCocos::get()
        .setup([] {
            auto& io = ImGui::GetIO();
            io.IniFilename = nullptr;
            applyTheme();
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
                auto btn = CCMenuItemExt::createSpriteExtra(spr, [](CCObject*) { g.showMain = !g.showMain; });
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
            auto btn = CCMenuItemExt::createSpriteExtra(spr, [](CCObject*) { g.showMain = !g.showMain; });
            btn->setPosition({winSize.width - 35.f, 35.f});
            menu->addChild(btn);
        }
        this->addChild(menu);
    }
};

// the pathfinder button on a level's page: a Geometry Dash cube
class $modify(PFLevelInfoLayer, LevelInfoLayer) {
    bool init(GJGameLevel* level, bool challenge) {
        if (!LevelInfoLayer::init(level, challenge)) return false;

        auto bg = CCSprite::createWithSpriteFrameName("GJ_button_01.png");
        if (!bg) return true;
        if (auto cube = SimplePlayer::create(1)) {
            cube->setScale(0.8f);
            cube->setPosition({bg->getContentSize().width * 0.5f, bg->getContentSize().height * 0.5f});
            bg->addChild(cube);
        }
        auto btn = CCMenuItemExt::createSpriteExtra(bg, [level](CCObject*) {
            g.setupLevel = level;
            g.showMain = false;
            g.showSetup = true;
        });
        btn->setID("pathfinder-level-button"_spr);

        if (auto side = typeinfo_cast<CCMenu*>(this->getChildByID("left-side-menu"))) {
            side->addChild(btn);
            side->updateLayout();
        } else {
            auto winSize = CCDirector::get()->getWinSize();
            auto menu = CCMenu::create();
            menu->setPosition({0, 0});
            btn->setPosition({34.f, winSize.height * 0.5f - 40.f});
            menu->addChild(btn);
            this->addChild(menu, 10);
        }
        return true;
    }
};

// ================================================================= hooks
class $modify(PFBaseGameLayer, GJBaseGameLayer) {
    void processCommands(float dt, bool isHalfTick, bool isLastTick) {
        auto pl = PlayLayer::get();
        if (pl && static_cast<GJBaseGameLayer*>(pl) == this) tick(pl);
        GJBaseGameLayer::processCommands(dt, isHalfTick, isLastTick);
    }

    // TPS above 240: stop quantizing the delta to 1/240 s ...
    float getModifiedDelta(float dt) {
        auto pl = PlayLayer::get();
        if (g.tps > 240 && pl && static_cast<GJBaseGameLayer*>(pl) == this) return dt;
        return GJBaseGameLayer::getModifiedDelta(dt);
    }

    // ... and feed the game one tiny delta per physics step. The vanilla step count is
    // max(1, delta*240), so any delta <= 1/240 s becomes exactly one step of that length.
    void update(float dt) {
        auto pl = PlayLayer::get();
        if (g.tps <= 240 || !pl || static_cast<GJBaseGameLayer*>(pl) != this) {
            GJBaseGameLayer::update(dt);
            return;
        }
        double step = 1.0 / g.tps;
        rt.accum += dt;
        int n = 0;
        while (rt.accum >= step && n < 20000) {
            GJBaseGameLayer::update((float)step);
            rt.accum -= step;
            n++;
        }
        if (n >= 20000) rt.accum = 0.0;   // can't keep up: drop the backlog
    }

    // Same place every bot records from, so frames match what Eclipse replays.
    void handleButton(bool down, int button, bool isPlayer1) {
        GJBaseGameLayer::handleButton(down, button, isPlayer1);

        auto pl = PlayLayer::get();
        if (!pl || static_cast<GJBaseGameLayer*>(pl) != this) return;
        if (button != 1 || !isPlayer1) return;   // this mod only drives / records P1
        rt.held = down;                          // real input state, used by the planner
        if (down) rt.lastPress = progress(this);

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
        rt.lastPress = -100000;
        rt.lastProg = -1;
        rt.accum = 0.0;
        rt.floorBottom = 90.0;
        rt.ceilTop = 390.0;

        mapLevel(this);

        rt.draw = CCDrawNode::create();
        m_objectLayer->addChild(rt.draw, 1000);

        auto winSize = CCDirector::get()->getWinSize();
        rt.label = CCLabelBMFont::create("", "bigFont.fnt");
        rt.label->setScale(0.38f);
        rt.label->setAnchorPoint({0.f, 1.f});
        rt.label->setPosition({8.f, winSize.height - 8.f});
        rt.label->setVisible(false);
        this->addChild(rt.label, 200);

        if (sess.armed) beginSession(this);
        else g.status = "Level loaded.";
    }

    void resetLevel() {
        PlayLayer::resetLevel();
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
        rt.lastPress = -100000;
        rt.lastProg = -1;
    }

    void destroyPlayer(PlayerObject* player, GameObject* obj) {
        bool wasDead = m_playerDied;
        PlayLayer::destroyPlayer(player, obj);
        if (wasDead || !m_playerDied || !sess.active) return;   // anti-cheat spike etc. don't count
        onFailure(this);
        if (obj) {
            auto pos = obj->getPosition();
            plog(fmt::format("    killed by object id {} at ({:.0f}, {:.0f})", (int)obj->m_objectID, pos.x, pos.y));
        }
    }

    void levelComplete() {
        PlayLayer::levelComplete();
        if (sess.active) {
            sess.attempt++;
            sess.best = 100.f;
            endSession(fmt::format("PASSED on attempt {}.", sess.attempt).c_str(), Phase::Passed);
            g.status = fmt::format("Passed on attempt {}.", sess.attempt);
        }
        if (g.record && g.autoSave && !rt.inputs.empty()) saveReplay(this);
    }

    void onQuit() {
        g.inLevel = false;
        rt.draw = nullptr;
        rt.label = nullptr;
        if (sess.active) endSession("Left the level - session stopped.", Phase::Idle);
        setTimeScale(1.f);
        PlayLayer::onQuit();
    }

    void postUpdate(float dt) {
        PlayLayer::postUpdate(dt);
        if (rt.draw) rt.draw->clear();
        g.percent = getCurrentPercent();

        if (rt.label) {
            rt.label->setVisible(sess.active);
            if (sess.active) {
                rt.label->setString(fmt::format("Pathfinder {:.0f}x | attempt {} | best {:.0f}% | now {:.0f}%",
                    g.speed, sess.attempt + 1, sess.best, g.percent).c_str());
            }
        }

        auto p = m_player1;
        if (!p || p->m_isDead) return;

        Mode m = modeOf(p);
        g.modeName = modeName(m);
        SimParams s = buildParams(p, m, progress(this));

        float px = p->getPositionX();
        double reach = s.vx * s.steps * kStepSeconds + 80.0;
        static std::vector<CCRect> hzW, solW;
        sliceMap(lmap.hz,  lmap.hzMaxW,  px - 40.f, (float)(px + reach), p->getPositionY(), hzW);
        sliceMap(lmap.sol, lmap.solMaxW, px - 40.f, (float)(px + reach), p->getPositionY(), solW);

        g.nextHazard = -1.f;
        for (auto const& r : hzW) {
            if (r.getMinX() >= px) { g.nextHazard = r.getMinX() - px; break; }
        }

        Policy none{0, 0, 0};
        g.inputSafe = survive(s, none, hzW, solW) >= s.steps;

        if (g.overlay && rt.draw) {
            for (auto const& r : hzW) rt.draw->drawDot({r.getMidX(), r.getMidY()}, 7.f, {1.f, 0.2f, 0.2f, 0.7f});
            std::vector<CCPoint> path;
            survive(s, none, hzW, solW, &path);
            ccColor4F col = g.inputSafe ? ccColor4F{0.2f, 1.f, 0.4f, 0.8f}
                                        : ccColor4F{1.f, 0.6f, 0.1f, 0.8f};
            for (auto const& pt : path) rt.draw->drawDot(pt, 2.5f, col);
        }
    }
};
