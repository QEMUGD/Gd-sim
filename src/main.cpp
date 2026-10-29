// Pathfinder - Geode mod for Geometry Dash 2.2081 (Windows + Android)
// Author: qemugd  https://github.com/qemugd/gd-sim
//
// Press the cube button on a level page, pick settings, press OK. The mod
//   1. maps the level (hazards, solid blocks, portals),
//   2. plans a whole-level macro (holds and taps) in a physics simulation,
//   3. verifies that macro in the simulation and retries with other margins if it fails,
//   4. plays the verified macro in the real game, re-planning if the run drifts,
//   5. on a death: logs it, widens the caution around it, re-maps and plans again,
//   6. when the level is passed, saves the inputs as a .gdr2 replay (Eclipse Menu etc.).
//
// Credits
//   * Physics model: camila314's GD physics decompilations (camila314/gdp,
//     2.2 branch, used with permission). Constants and formulas below come from
//       - PlayerObject::updateTimeMod  (yStart / gravity / speed per speed portal)
//       - PlayerObject::updateJump     (jump rules for every vehicle, mini scaling,
//                                       gravity formula, fall clamps, robot boost)
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
static constexpr double kDt = 0.25;           // one 240 Hz step, in 1/60 s "frame units"
static constexpr double kStepSeconds = 1.0 / 240.0;
static constexpr double kFallClamp = 15.0;    // ground-vehicle fall clamp
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
static bool groundMode(Mode m) { return m == Mode::Cube || m == Mode::Ball || m == Mode::Robot || m == Mode::Spider; }

// level events found by mapping (portals), identified by their fixed object IDs
enum class EvtType { SetMode, SetGravity, SetMini, SetSpeed };
struct Evt { float x, minY, maxY; EvtType type; int ival; double dval; };

struct Zone { float x0, x1; double extra; int fails; };   // extra caution learned from failures

struct Env {
    std::vector<CCRect> const* hz = nullptr;    // sorted by minX
    std::vector<CCRect> const* sol = nullptr;   // sorted by minX
    std::vector<Evt> const* evts = nullptr;     // sorted by x
    std::vector<Zone> const* zones = nullptr;
    bool   useZones = true;
    double floorBottom = 90.0;     // lowest allowed rect minY
    double ceilTop = 390.0;        // highest allowed rect maxY
    double gmod = 1.0;
    float  margin = 2.f;           // extra clearance around hazards
    int    lockSteps = 0;          // CPS lock: min 240 Hz steps between presses (0 = off)
    bool   allowHold = true;
    double xEnd = 1e9;
};

// one vehicle state; x/y are the player rect's minX/minY
struct SState {
    double x = 0, y = 0, w = 30, h = 30;
    double vy = 0, flip = 1, accel = 0, speed = 0.9;
    Mode mode = Mode::Cube;
    bool mini = false, grounded = true, down = false;
    int lastPress = -100000;
    int nextEvt = 0;
};

static float marginAt(Env const& e, double x) {
    float m = e.margin;
    if (e.useZones && e.zones) {
        float extra = 0.f;
        for (auto const& z : *e.zones) if (x >= z.x0 && x <= z.x1) extra = std::max(extra, (float)z.extra);
        m += extra;
    }
    return m;
}

// Advances one 240 Hz physics step with the input `down` held. Returns false on death.
static bool stepSim(SState& s, bool down, Env const& e, int stepNo) {
    bool pressed = down && !s.down;
    if (pressed) s.lastPress = stepNo;
    s.down = down;

    auto prof = profileFor((float)s.speed);
    double jumpScale = s.mini ? 0.8 : 1.0, v16 = s.mini ? 0.85 : 1.0;
    double gBase = prof.gravity * e.gmod, gAlt = kFlyGravity * e.gmod;
    double vxu = s.speed * prof.speedMult * 60.0;

    auto clampFly = [&](double lo, double hi) {   // gravity-relative clamp
        double u = s.vy * s.flip;
        if (u < lo) u = lo;
        if (u > hi) u = hi;
        s.vy = u * s.flip;
    };
    auto clampFall = [&]() { if (s.vy * s.flip < -kFallClamp) s.vy = -kFallClamp * s.flip; };

    double floorY = e.floorBottom, ceilY = e.ceilTop - s.h;
    bool trig = pressed || (down && s.grounded);   // holding re-triggers on landing

    switch (s.mode) {
    case Mode::Cube:
        if (trig && s.grounded) { s.vy = s.flip * prof.yStart * jumpScale; s.grounded = false; }
        else if (!s.grounded)   { s.vy -= gBase * kDt * s.flip; clampFall(); }
        break;
    case Mode::Ball:
        if (trig && s.grounded) {
            s.vy = s.flip * prof.yStart * jumpScale;
            s.flip = -s.flip;
            s.vy *= 0.6;
            s.grounded = false;
        } else if (!s.grounded) { s.vy -= gAlt * kDt * s.flip * 0.6; clampFall(); }
        break;
    case Mode::Robot:
        if ((pressed || down) && s.grounded) {
            s.vy = s.flip * prof.yStart * 0.5 * jumpScale; s.grounded = false; s.accel = 0;
        } else if (!s.grounded) {
            if (down && s.accel < 1.5) s.accel += kDt / 10.0;      // gravity cancelled while boosting
            else { s.vy -= gBase * kDt * s.flip * 0.9; clampFall(); }
        }
        break;
    case Mode::Spider:
        if (trig && s.grounded) {                // teleports to the opposite surface
            s.flip = -s.flip;
            s.y = s.flip > 0 ? floorY : ceilY;
            s.vy = 0; s.grounded = true;
        } else if (!s.grounded) { s.vy -= gAlt * kDt * s.flip * 0.6; clampFall(); }
        break;
    case Mode::Ship: {
        double dv = down ? (gAlt * kDt * 0.4 / v16) : (-1.2 * gAlt * kDt * 0.4 / v16);
        s.vy += dv * s.flip;
        clampFly(-6.4 / v16, 8.0 / v16);
        break;
    }
    case Mode::Ufo: {
        if (pressed) {
            double v = (s.mini ? 8.0 : 7.0) * v16;
            if (v > s.vy * s.flip) s.vy = s.flip * v;
        }
        s.vy -= gAlt * kDt * s.flip * 1.2 * 0.5 / v16;
        clampFly(-6.4 / v16, 8.0 / v16);
        break;
    }
    case Mode::Wave:
        s.vy = s.flip * s.speed * prof.speedMult * (down ? 1.0 : -1.0);
        break;
    case Mode::Swing:
        if (pressed) { s.vy *= 0.8; s.flip = -s.flip; }
        s.vy -= gAlt * kDt * s.flip * (s.mini ? 0.6 : 0.4);
        clampFly(-8.0, 8.0);
        break;
    }

    if (!s.grounded) s.y += s.vy * kDt;
    s.x += vxu * kStepSeconds;

    // level floor / ceiling: land on them (ground vehicles) or slide along them
    bool supported = false;
    if (s.y <= floorY) {
        s.y = floorY; if (s.vy < 0) s.vy = 0;
        if (s.flip > 0) { supported = true; if (groundMode(s.mode)) s.grounded = true; }
    }
    if (s.y >= ceilY) {
        s.y = ceilY; if (s.vy > 0) s.vy = 0;
        if (s.flip < 0) { supported = true; if (groundMode(s.mode)) s.grounded = true; }
    }

    // solid blocks: land on top, bump underneath, die on the sides
    double rMinX = s.x, rMaxX = s.x + s.w;
    if (e.sol) {
        auto const& sol = *e.sol;
        auto it = std::lower_bound(sol.begin(), sol.end(), (float)(rMinX - 120.0),
            [](CCRect const& r, float v) { return r.getMinX() < v; });
        for (; it != sol.end() && it->getMinX() <= rMaxX + 2; ++it) {
            auto const& b = *it;
            if (b.getMaxX() < rMinX - 2) continue;
            double rMinY = s.y, rMaxY = s.y + s.h;
            bool hOver = rMaxX > b.getMinX() + 0.5 && rMinX < b.getMaxX() - 0.5;
            if (!hOver) continue;
            if (s.grounded) {   // resting contact keeps us supported
                if (s.flip > 0 && std::abs(b.getMaxY() - rMinY) <= 2.0) supported = true;
                if (s.flip < 0 && std::abs(b.getMinY() - rMaxY) <= 2.0) supported = true;
            }
            bool vOver = rMaxY > b.getMinY() + 1.0 && rMinY < b.getMaxY() - 1.0;
            if (!vOver) continue;
            if (s.mode == Mode::Wave) return false;              // wave dies on any block
            double tol = std::max(4.0, std::abs(s.vy) * kDt + 2.0);
            double feetPen = s.flip > 0 ? b.getMaxY() - rMinY : rMaxY - b.getMinY();
            double headPen = s.flip > 0 ? rMaxY - b.getMinY() : b.getMaxY() - rMinY;
            if (feetPen <= tol && s.vy * s.flip <= 0.0) {
                s.y += s.flip * feetPen; s.vy = 0; supported = true;
                if (groundMode(s.mode)) s.grounded = true;
            } else if (headPen <= tol && s.vy * s.flip >= 0.0) {
                s.y -= s.flip * headPen; if (s.vy * s.flip > 0) s.vy = 0;
            } else {
                return false;                                    // side hit
            }
        }
    }
    if (s.grounded && !supported) { s.grounded = false; s.vy = 0; } // ran off an edge

    // portals / events we pass
    if (e.evts) {
        auto const& ev = *e.evts;
        double cx = s.x + s.w * 0.5;
        while (s.nextEvt < (int)ev.size() && cx >= ev[s.nextEvt].x) {
            auto const& v = ev[s.nextEvt++];
            if (!(s.y + s.h > v.minY && s.y < v.maxY)) continue;  // flew past without touching it
            switch (v.type) {
                case EvtType::SetMode:
                    s.mode = (Mode)v.ival; s.grounded = false; s.accel = 0; break;
                case EvtType::SetGravity:
                    s.flip = v.ival; s.grounded = false; break;
                case EvtType::SetMini: {
                    bool mini = v.ival != 0;
                    if (mini != s.mini) {
                        double k = mini ? 0.6 : 1.0 / 0.6;
                        double cxm = s.x + s.w * 0.5, cym = s.y + s.h * 0.5;
                        s.w *= k; s.h *= k;
                        s.x = cxm - s.w * 0.5; s.y = cym - s.h * 0.5;
                        s.mini = mini;
                    }
                    break;
                }
                case EvtType::SetSpeed: s.speed = v.dval; break;
            }
        }
    }

    // hazards
    if (e.hz) {
        auto const& hz = *e.hz;
        float m = marginAt(e, s.x);
        CCRect r((float)s.x, (float)s.y, (float)s.w, (float)s.h);
        auto it = std::lower_bound(hz.begin(), hz.end(), (float)(s.x - 130.0),
            [](CCRect const& q, float v) { return q.getMinX() < v; });
        for (; it != hz.end() && it->getMinX() <= rMaxX + m + 2; ++it) {
            if (it->getMaxX() < rMinX - m - 2) continue;
            CCRect t(it->origin.x - m, it->origin.y - m, it->size.width + 2 * m, it->size.height + 2 * m);
            if (r.intersectsRect(t)) return false;
        }
    }
    return true;
}

// ------------------------------------------------------------------ planner
// Forward reachability over "hold / release" choices, one choice per G steps.
// Distinct states are merged, so the whole level is searched in one sweep.
struct PlanOut {
    bool reachedEnd = false;
    int  layers = 0;
    double reachX = 0;
    std::vector<uint8_t> down;   // one entry per G steps
};

static uint64_t mix64(uint64_t h, uint64_t v) {
    h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
}

static PlanOut planMacro(SState const& start, Env const& e, int G, int cap, int variant,
                         int maxLayers, size_t maxNodes = 6000000) {
    struct Node { int parent; uint8_t down; };
    struct Cand { SState st; int node; int cost; };
    std::vector<Node> nodes; nodes.reserve(1 << 16);
    std::vector<Cand> cur, next;
    std::vector<uint64_t> keys;       // parallel to `next`: merge key of each state
    cur.push_back({start, -1, 0});
    PlanOut out;
    int bestNode = -1;
    double bestX = start.x;
    bool order[2] = {(variant % 2) != 0, (variant % 2) == 0};   // which choice to expand first

    for (int layer = 0; layer < maxLayers; ++layer) {
        next.clear(); keys.clear();
        int stepNo = layer * G;
        for (auto const& c : cur) {
            for (int k = 0; k < 2; k++) {
                bool act = order[k];
                bool press = act && !c.st.down;
                if (press && e.lockSteps > 0 && stepNo - c.st.lastPress < e.lockSteps) continue;
                if (act && c.st.down && !e.allowHold) continue;
                SState t = c.st;
                bool alive = true;
                for (int i = 0; i < G && alive; i++) alive = stepSim(t, act, e, stepNo + i);
                if (!alive) continue;

                uint64_t key = 1469598103934665603ULL;
                key = mix64(key, (uint64_t)(int64_t)std::llround(t.y));
                key = mix64(key, (uint64_t)(int64_t)std::llround(t.vy * 2.0));
                key = mix64(key, (uint64_t)((t.flip > 0 ? 1 : 2) | ((int)t.mode << 2) | (t.grounded ? 64 : 0) |
                                            (t.down ? 128 : 0) | (t.mini ? 256 : 0)));
                key = mix64(key, (uint64_t)std::llround(t.accel * 4.0));
                key = mix64(key, (uint64_t)std::min(t.lastPress >= -99999 ? stepNo + G - t.lastPress : 99999,
                                                    std::max(e.lockSteps, 1)));
                key = mix64(key, (uint64_t)t.nextEvt);
                int cost = c.cost + (press ? 10 : 0) + (act ? 1 : 0);   // prefer few presses, short holds
                int dup = -1;
                for (int q = 0; q < (int)keys.size(); q++) if (keys[q] == key) { dup = q; break; }
                nodes.push_back({c.node, (uint8_t)(act ? 1 : 0)});
                if (dup >= 0) {                                   // merged duplicate: keep the cheaper path
                    if (cost < next[dup].cost) next[dup] = {t, (int)nodes.size() - 1, cost};
                    continue;
                }
                keys.push_back(key);
                next.push_back({t, (int)nodes.size() - 1, cost});
            }
        }
        if (next.empty()) break;                         // stuck: no safe continuation
        out.layers = layer + 1;

        double maxX = -1e18; int maxI = 0;
        for (int i = 0; i < (int)next.size(); i++) {
            if (next[i].st.x > maxX + 1e-6 || (std::abs(next[i].st.x - maxX) <= 1e-6 && next[i].cost < next[maxI].cost)) {
                maxX = next[i].st.x; maxI = i;
            }
        }
        bestNode = next[maxI].node; bestX = maxX;
        if (maxX >= e.xEnd) { out.reachedEnd = true; break; }
        if (nodes.size() > maxNodes) break;

        if ((int)next.size() > cap) {
            // keep the cheapest half outright, and an even spread of the rest for diversity
            int keepCheap = cap / 2;
            std::partial_sort(next.begin(), next.begin() + keepCheap, next.end(),
                [](Cand const& a, Cand const& b) { return a.cost < b.cost; });
            std::vector<Cand> kept(next.begin(), next.begin() + keepCheap);
            std::vector<Cand> rest(next.begin() + keepCheap, next.end());
            std::sort(rest.begin(), rest.end(), [](Cand const& a, Cand const& b) {
                if (std::llround(a.st.y) != std::llround(b.st.y)) return a.st.y < b.st.y;
                return a.st.vy < b.st.vy;
            });
            int want = cap - keepCheap;
            double stepf = (double)rest.size() / want;
            for (int i = 0; i < want; i++) kept.push_back(rest[(size_t)(i * stepf + (variant % 3) * stepf / 3.0)]);
            next.swap(kept);
        }
        cur.swap(next);
    }

    out.reachX = bestX;
    for (int n = bestNode; n >= 0; n = nodes[n].parent) out.down.push_back(nodes[n].down);
    std::reverse(out.down.begin(), out.down.end());
    return out;
}

// Independent check: replays a finished macro step by step (no zone padding, margin 0).
// Optionally records predicted positions every 12 steps. Returns true if it reaches xEnd/ the
// end of the macro alive; failStep gets the step of death otherwise.
static bool verifyMacro(SState start, Env e, std::vector<uint8_t> const& down, int G,
                        std::vector<CCPoint>* path, std::vector<float>* predMinY,
                        int* failStep = nullptr) {
    e.useZones = false;
    e.margin = 0.f;
    int step = 0;
    for (size_t b = 0; b < down.size(); b++) {
        for (int i = 0; i < G; i++, step++) {
            if (path && step % 12 == 0) {
                path->push_back({(float)(start.x + start.w * 0.5), (float)(start.y + start.h * 0.5)});
                if (predMinY) predMinY->push_back((float)start.y);
            }
            if (!stepSim(start, down[b] != 0, e, step)) { if (failStep) *failStep = step; return false; }
        }
    }
    return true;
}
// ==== SIM END

// ================================================================= state
static constexpr int kG = 2;   // the planner chooses hold/release once per kG physics steps (240 Hz)

struct PFState {
    // ---- settings (saved)
    bool  record       = true;
    bool  autoSave     = true;
    bool  overlay      = true;
    bool  allowHold    = true;    // allow held inputs (otherwise taps only)
    int   beam         = 256;     // planner width (states kept per step)
    float margin       = 2.f;     // extra clearance around hazards
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
    int   clicks       = 0;
    bool  dirty        = false;
    float percent      = 0.f;
    char const* modeName = "-";
    std::string status = "Ready.";
    std::string planInfo = "No plan yet.";
};
static PFState g;

enum class Phase { Idle, Mapping, Botting, Passed, GaveUp };
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
    std::vector<Evt> evts;         // sorted by x
    float xEnd = 0;
    int unsupported = 0;           // orbs / pads / dual / mirror we do not model
};
static LevelMap lmap;

struct Plan {
    bool valid = false, full = false;
    int  startProg = 0;
    std::vector<uint8_t> down;     // one entry per kG steps
    std::vector<CCPoint> path;     // predicted centre every 12 steps
    std::vector<float> predY;      // predicted rect minY every 12 steps
    int  inputs = 0, holds = 0;
    double ms = 0;
};

struct Runtime {
    std::vector<std::pair<uint64_t, bool>> inputs; // (frame, down) as recorded
    Plan   plan;
    bool   held = false;                  // real P1 input state
    int    lastPress = -100000;
    int    lastProg = -1;
    int    lastReplan = -100000;
    int    replans = 0;
    double floorBottom = 90.0;            // level bounds (game coords)
    double ceilTop     = 390.0;
    double gmod = 1.0;
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
    m->setSavedValue("record", g.record);     m->setSavedValue("autosave", g.autoSave);
    m->setSavedValue("overlay", g.overlay);   m->setSavedValue("allowhold", g.allowHold);
    m->setSavedValue("beam", g.beam);         m->setSavedValue("margin", g.margin);
    m->setSavedValue("speed", g.speed);       m->setSavedValue("tps", g.tps);
    m->setSavedValue("cpslock", g.cpsLock);   m->setSavedValue("cps", g.cps);
    m->setSavedValue("maxattempts", g.maxAttempts);
}
static void loadSettings() {
    auto m = Mod::get();
    g.record = m->getSavedValue<bool>("record", true);
    g.autoSave = m->getSavedValue<bool>("autosave", true);
    g.overlay = m->getSavedValue<bool>("overlay", true);
    g.allowHold = m->getSavedValue<bool>("allowhold", true);
    g.beam = m->getSavedValue<int>("beam", 256);
    g.margin = m->getSavedValue<float>("margin", 2.f);
    g.speed = m->getSavedValue<float>("speed", 10.f);
    g.tps = m->getSavedValue<int>("tps", 240);
    g.cpsLock = m->getSavedValue<bool>("cpslock", true);
    g.cps = m->getSavedValue<int>("cps", 16);
    g.maxAttempts = m->getSavedValue<int>("maxattempts", 300);
    g.tps = std::clamp(g.tps, 240, 5000);
    g.speed = std::clamp(g.speed, 1.f, 10.f);
    g.beam = std::clamp(g.beam, 32, 1024);
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

// portals are recognised by their fixed object IDs
static bool portalEvent(int id, Evt& ev) {
    ev.ival = 0; ev.dval = 0;
    switch (id) {
        case 12:   ev.type = EvtType::SetMode; ev.ival = (int)Mode::Cube;   return true;
        case 13:   ev.type = EvtType::SetMode; ev.ival = (int)Mode::Ship;   return true;
        case 47:   ev.type = EvtType::SetMode; ev.ival = (int)Mode::Ball;   return true;
        case 111:  ev.type = EvtType::SetMode; ev.ival = (int)Mode::Ufo;    return true;
        case 660:  ev.type = EvtType::SetMode; ev.ival = (int)Mode::Wave;   return true;
        case 745:  ev.type = EvtType::SetMode; ev.ival = (int)Mode::Robot;  return true;
        case 1331: ev.type = EvtType::SetMode; ev.ival = (int)Mode::Spider; return true;
        case 1933: ev.type = EvtType::SetMode; ev.ival = (int)Mode::Swing;  return true;
        case 10:   ev.type = EvtType::SetGravity; ev.ival = 1;  return true;
        case 11:   ev.type = EvtType::SetGravity; ev.ival = -1; return true;
        case 99:   ev.type = EvtType::SetMini; ev.ival = 0; return true;
        case 101:  ev.type = EvtType::SetMini; ev.ival = 1; return true;
        case 200:  ev.type = EvtType::SetSpeed; ev.dval = 0.7; return true;
        case 201:  ev.type = EvtType::SetSpeed; ev.dval = 0.9; return true;
        case 202:  ev.type = EvtType::SetSpeed; ev.dval = 1.1; return true;
        case 203:  ev.type = EvtType::SetSpeed; ev.dval = 1.3; return true;
        case 1334: ev.type = EvtType::SetSpeed; ev.dval = 1.6; return true;
        default: return false;
    }
}
// orbs, pads, dual / mirror portals: not modelled, only counted
static bool isUnsupported(int id) {
    switch (id) {
        case 35: case 36: case 67: case 84: case 140: case 141: case 1022: case 1330:
        case 1332: case 1333: case 286: case 287: case 45: case 46: return true;
        default: return false;
    }
}

// Scans every object once: static hitboxes of hazards and solid blocks, plus portals.
static void mapLevel(PlayLayer* pl) {
    lmap.hz.clear(); lmap.sol.clear(); lmap.evts.clear();
    lmap.xEnd = 0.f; lmap.unsupported = 0;
    for (auto obj : CCArrayExt<GameObject*>(pl->m_objects)) {
        if (!obj) continue;
        lmap.xEnd = std::max(lmap.xEnd, (float)obj->getPositionX());
        int id = (int)obj->m_objectID;
        if (isUnsupported(id)) lmap.unsupported++;

        Evt ev{};
        if (portalEvent(id, ev)) {
            CCRect r = obj->getObjectRect();
            ev.x = (float)obj->getPositionX();
            ev.minY = r.getMinY();
            ev.maxY = r.getMaxY();
            lmap.evts.push_back(ev);
            continue;
        }
        bool hazard = obj->m_objectType == GameObjectType::Hazard;
        bool solid  = obj->m_objectType == GameObjectType::Solid;
        if (!hazard && !solid) continue;
        CCRect r = obj->getObjectRect();
        if (r.size.width <= 0.f || r.size.height <= 0.f) continue;
        (hazard ? lmap.hz : lmap.sol).push_back(r);
    }
    auto byX = [](CCRect const& a, CCRect const& b) { return a.getMinX() < b.getMinX(); };
    std::sort(lmap.hz.begin(),  lmap.hz.end(),  byX);
    std::sort(lmap.sol.begin(), lmap.sol.end(), byX);
    std::sort(lmap.evts.begin(), lmap.evts.end(), [](Evt const& a, Evt const& b) { return a.x < b.x; });
}

// ================================================================= planning
static void learnBounds(PlayerObject* p, Mode m) {
    // Only trust surface contact in modes that live between floor and ceiling.
    if (m == Mode::Cube || m == Mode::Robot || m == Mode::Spider) return;
    if (!p->m_isOnGround) return;
    auto r = p->getObjectRect();
    if (p->m_isUpsideDown) rt.ceilTop = r.getMaxY();
    else                   rt.floorBottom = r.getMinY();
}

static SState startState(PlayerObject* p, int prog) {
    SState s;
    auto r = p->getObjectRect();
    s.x = r.getMinX(); s.y = r.getMinY(); s.w = r.size.width; s.h = r.size.height;
    s.vy = p->m_yVelocity;
    s.flip = p->m_isUpsideDown ? -1.0 : 1.0;
    s.speed = p->m_playerSpeed;
    s.mode = modeOf(p);
    s.mini = p->m_vehicleSize != 1.f;
    s.grounded = p->m_isOnGround;
    s.down = rt.held;
    s.accel = 0;
    double since = rt.lastPress < -50000 ? 100000.0 : (prog - rt.lastPress) * 240.0 / std::max(1, g.tps);
    s.lastPress = -(int)std::min(100000.0, since);
    auto it = std::lower_bound(lmap.evts.begin(), lmap.evts.end(), (float)(s.x + s.w * 0.5),
        [](Evt const& e, float v) { return e.x < v; });
    s.nextEvt = (int)(it - lmap.evts.begin());
    return s;
}

static Env makeEnv(float marginScale, bool zones) {
    Env e;
    e.hz = &lmap.hz; e.sol = &lmap.sol; e.evts = &lmap.evts; e.zones = &sess.zones;
    e.useZones = zones;
    e.floorBottom = rt.floorBottom; e.ceilTop = rt.ceilTop;
    e.gmod = rt.gmod;
    e.margin = g.margin * marginScale;
    e.lockSteps = (g.cpsLock && g.cps > 0) ? (int)std::lround(240.0 / g.cps) : 0;
    e.allowHold = g.allowHold;
    e.xEnd = lmap.xEnd;
    return e;
}

static void fillPlan(Plan& pl, std::vector<uint8_t> const& down, bool full, int startProg, double ms,
                     std::vector<CCPoint>&& path, std::vector<float>&& predY) {
    pl.valid = true; pl.full = full; pl.startProg = startProg; pl.ms = ms;
    pl.down = down; pl.path = std::move(path); pl.predY = std::move(predY);
    pl.inputs = 0; pl.holds = 0;
    int run = 0; bool prev = false;
    for (size_t i = 0; i <= down.size(); i++) {
        bool d = i < down.size() && down[i];
        if (d && !prev) { pl.inputs++; run = 0; }
        if (d) run++;
        if (!d && prev && run > 2) pl.holds++;
        prev = d;
    }
}

// Plans from the player's real state: plan -> verify -> retry with other margins.
static bool makePlan(PlayLayer* pl, char const* why) {
    auto p = pl->m_player1;
    if (!p) return false;
    int prog = progress(pl);
    rt.gmod = p->m_gravityMod;
    sess.phase = Phase::Mapping;

    SState st = startState(p, prog);
    double minV = 0.7 * 5.980002 * 60.0 / 240.0;                       // slowest speed, units per step
    int maxLayers = (int)(std::max(0.0, lmap.xEnd + 120.0 - st.x) / minV / kG) + 400;

    struct Try { float scale; bool zones; };
    Try tries[3] = {{1.f, true}, {0.5f, true}, {0.f, false}};
    PlanOut bestPartial; bool havePartial = false; Env partialEnv;

    for (int t = 0; t < 3; t++) {
        Env env = makeEnv(tries[t].scale, tries[t].zones);
        auto t0 = std::chrono::steady_clock::now();
        PlanOut out = planMacro(st, env, kG, g.beam, sess.attempt + t, maxLayers);
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        if (out.reachedEnd) {
            std::vector<CCPoint> path; std::vector<float> predY; int failStep = -1;
            bool ok = verifyMacro(st, env, out.down, kG, &path, &predY, &failStep);
            if (ok) {
                fillPlan(rt.plan, out.down, true, prog, ms, std::move(path), std::move(predY));
                g.planInfo = fmt::format("Verified macro: {} presses, {} holds, planned in {:.0f} ms (margin {:.1f}).",
                                         rt.plan.inputs, rt.plan.holds, ms, env.margin);
                plog(fmt::format("[plan:{}] try {} (margin {:.1f}): reached the end in {:.0f} ms | verification PASSED | {} presses, {} holds",
                                 why, t + 1, env.margin, ms, rt.plan.inputs, rt.plan.holds));
                sess.phase = Phase::Botting;
                return true;
            }
            plog(fmt::format("[plan:{}] try {} (margin {:.1f}): reached the end but verification FAILED at step {} - retrying",
                             why, t + 1, env.margin, failStep));
        } else {
            plog(fmt::format("[plan:{}] try {} (margin {:.1f}): no safe path past x={:.0f} (level end {:.0f}) - retrying",
                             why, t + 1, env.margin, out.reachX, lmap.xEnd));
            if (!havePartial || out.reachX > bestPartial.reachX) {
                bestPartial = std::move(out); havePartial = true; partialEnv = env;
            }
        }
    }

    // nothing verified: run the furthest partial plan so the log shows where the model gives up
    if (havePartial && !bestPartial.down.empty()) {
        std::vector<CCPoint> path; std::vector<float> predY;
        verifyMacro(st, partialEnv, bestPartial.down, kG, &path, &predY);
        fillPlan(rt.plan, bestPartial.down, false, prog, 0.0, std::move(path), std::move(predY));
        g.planInfo = fmt::format("No complete macro found. Running the best partial plan (to x={:.0f}).", bestPartial.reachX);
        plog(fmt::format("[plan:{}] NO COMPLETE MACRO. Running partial plan to x={:.0f}.", why, bestPartial.reachX));
    } else {
        rt.plan = Plan{};
        g.planInfo = "No plan could be made.";
        plog(fmt::format("[plan:{}] NO PLAN: the model sees no safe move from here.", why));
    }
    sess.phase = Phase::Botting;
    return false;
}

// ================================================================= playback
static void tick(PlayLayer* pl) {
    int prog = progress(pl);
    if (prog == rt.lastProg) return;            // one decision per physics step
    rt.lastProg = prog;

    auto p = pl->m_player1;
    if (!p || p->m_isDead) return;
    learnBounds(p, modeOf(p));
    if (!sess.active || !rt.plan.valid) return;

    double idx = (prog - rt.plan.startProg) * 240.0 / std::max(1, g.tps);   // 240 Hz step index
    if (idx < 0) return;
    int step = (int)idx;

    // has the real run drifted away from the prediction? then plan again from the real state
    if (step % 12 == 0 && (size_t)(step / 12) < rt.plan.predY.size() && prog - rt.lastReplan > 24) {
        float realY = p->getObjectRect().getMinY();
        float predY = rt.plan.predY[step / 12];
        if (std::abs(realY - predY) > 10.f && rt.replans < 30) {
            rt.replans++;
            rt.lastReplan = prog;
            plog(fmt::format("Drift at x={:.0f}: real y {:.0f} vs predicted {:.0f} - re-planning ({}).",
                             p->getPositionX(), realY, predY, rt.replans));
            makePlan(pl, "drift");
            step = 0;
        }
    }

    size_t block = (size_t)step / kG;
    bool want = block < rt.plan.down.size() ? rt.plan.down[block] != 0 : false;
    if (want != rt.held) {
        pl->handleButton(want, 1, true);       // recorded by our handleButton hook
        if (want) g.clicks++;
    }
}

// ================================================================= pathfinder session
static void beginSession(PlayLayer* pl) {
    sess.armed = false;
    sess.active = true;
    sess.phase = Phase::Mapping;
    sess.attempt = 0;
    sess.best = 0.f;
    sess.zones.clear();
    plog(fmt::format("=== Pathfinder session: {} (ID {}) | {:.1f}x speed, {} TPS, CPS lock {}, holding {} ===",
        std::string(pl->m_level->m_levelName.c_str()), (int)pl->m_level->m_levelID.value(),
        g.speed, g.tps, g.cpsLock ? std::to_string(g.cps) : std::string("off"), g.allowHold ? "on" : "off"));
    mapLevel(pl);
    plog(fmt::format("Mapped level: {} hazards, {} solid blocks, {} portals, end at x={:.0f}.",
        lmap.hz.size(), lmap.sol.size(), lmap.evts.size(), lmap.xEnd));
    if (lmap.unsupported > 0) {
        plog(fmt::format("Warning: {} orbs/pads/dual/mirror objects are not modelled; sections needing them will fail.",
            lmap.unsupported));
    }
    rt.replans = 0;
    makePlan(pl, "start");
    setTimeScale(g.speed);
}

static void endSession(std::string const& why, Phase result) {
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

    // add caution around the spot where the real run died
    Zone* hit = nullptr;
    for (auto& z : sess.zones) if (x >= z.x0 && x <= z.x1) { hit = &z; break; }
    if (hit) {
        hit->fails++;
        hit->extra = std::min(6.0, hit->extra + 1.0);
    } else {
        sess.zones.push_back({x - 80.f, x + 40.f, 1.0, 1});
        hit = &sess.zones.back();
    }
    plog(fmt::format("[attempt {}] FAILED at {:.1f}% (x={:.0f}, {}) | caution +{:.0f} here, fails here {}",
        sess.attempt, pct, x, p ? modeName(modeOf(p)) : "?", hit->extra, hit->fails));

    mapLevel(pl);   // re-map before the next try
    plog(fmt::format("Re-mapped ({} hazards, {} blocks, {} portals). Planning again...",
        lmap.hz.size(), lmap.sol.size(), lmap.evts.size()));

    if (sess.attempt >= g.maxAttempts) {
        endSession(fmt::format("Gave up after {} attempts (best {:.1f}%).", sess.attempt, sess.best), Phase::GaveUp);
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
    ImGui::Checkbox("Allow holding", &g.allowHold);
    ImGui::TextDisabled("Holds are used for ship, wave, robot and cube re-jumps. Off = taps only.");
    ImGui::SeparatorText("Pathfinding");
    ImGui::SliderInt("Max attempts", &g.maxAttempts, 1, 2000);
    ImGui::SliderInt("Planner width", &g.beam, 32, 1024);
    ImGui::SliderFloat("Safety margin", &g.margin, 0.f, 8.f, "%.1f");
    ImGui::SeparatorText("Output");
    ImGui::Checkbox("Record inputs (.gdr2)", &g.record);
    ImGui::Checkbox("Auto-save when passed", &g.autoSave);
    ImGui::Checkbox("Show hazards + planned path", &g.overlay);
}

static char const* phaseName(Phase p) {
    switch (p) {
        case Phase::Idle: return "Idle";       case Phase::Mapping: return "Mapping / planning";
        case Phase::Botting: return "Botting"; case Phase::Passed: return "Passed";
        case Phase::GaveUp: return "Gave up";
    }
    return "?";
}

static void drawSetupWindow() {
    auto ds = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos({ds.x * 0.5f, ds.y * 0.5f}, ImGuiCond_Always, {0.5f, 0.5f});
    ImGui::SetNextWindowSize({std::min(460.f, ds.x * 0.92f), std::min(640.f, ds.y * 0.92f)}, ImGuiCond_Always);
    ImGui::Begin("Pathfinder setup", nullptr,
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoTitleBar);
    drawHeader("PATHFINDER", "Map, plan, verify, then bot it");

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
    ImGui::SetNextWindowSize({420, 580}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos({30, 30}, ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Pathfinder", &g.showMain, ImGuiWindowFlags_NoTitleBar)) { ImGui::End(); return; }
    drawHeader("PATHFINDER", "by qemugd  |  physics: camila314/gdp");

    if (ImGui::BeginTabBar("pf_tabs")) {
        if (ImGui::BeginTabItem("Status")) {
            ImGui::Text("Phase: %s", phaseName(sess.phase));
            if (g.inLevel) {
                ImGui::Text("Mode: %s", g.modeName);
                ImGui::Text("Attempt: %d   Best: %.1f%%   Re-plans: %d", sess.attempt, sess.best, rt.replans);
                ImGui::ProgressBar(std::clamp(g.percent / 100.f, 0.f, 1.f), {-1, 0},
                    fmt::format("{:.1f}%", g.percent).c_str());
                ImGui::TextWrapped("%s", g.planInfo.c_str());
                ImGui::Text("Inputs recorded: %d  (presses: %d)", (int)rt.inputs.size(), g.clicks);
                if (g.dirty) ImGui::TextColored({1.f, 0.7f, 0.2f, 1.f}, "Run has practice respawns");
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
            ImGui::Text("Portals mapped: %d", (int)lmap.evts.size());
            ImGui::Text("Not modelled (orbs/pads/dual/mirror): %d", lmap.unsupported);
            ImGui::Text("Level end: x=%.0f", lmap.xEnd);
            ImGui::Text("Floor %.0f  |  Ceiling %.0f", rt.floorBottom, rt.ceilTop);
            ImGui::SeparatorText("Caution zones (from failures)");
            if (sess.zones.empty()) ImGui::TextDisabled("none yet");
            for (auto const& z : sess.zones) {
                ImGui::BulletText("x %.0f-%.0f  +%.0f margin  fails %d", z.x0, z.x1, z.extra, z.fails);
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
        rt.held = down;                          // real input state
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
        g.clicks = 0;
        g.dirty = false;
        rt.inputs.clear();
        rt.plan = Plan{};
        rt.held = false;
        rt.lastPress = -100000;
        rt.lastProg = -1;
        rt.lastReplan = -100000;
        rt.replans = 0;
        rt.accum = 0.0;
        rt.floorBottom = 90.0;
        rt.ceilTop = 390.0;
        g.planInfo = "No plan yet.";

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
            g.clicks = 0;
        }
        rt.held = false;
        rt.lastPress = -100000;
        rt.lastProg = -1;
        rt.lastReplan = -100000;
        rt.replans = 0;
        if (sess.active) makePlan(this, "retry");   // fresh plan for the new attempt
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
            endSession(fmt::format("PASSED on attempt {}.", sess.attempt), Phase::Passed);
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
        g.modeName = modeName(modeOf(p));

        if (g.overlay && rt.draw) {
            float px = p->getPositionX();
            auto it = std::lower_bound(lmap.hz.begin(), lmap.hz.end(), px - 60.f,
                [](CCRect const& r, float v) { return r.getMinX() < v; });
            for (int n = 0; it != lmap.hz.end() && it->getMinX() < px + 600.f && n < 60; ++it, ++n) {
                rt.draw->drawDot({it->getMidX(), it->getMidY()}, 7.f, {1.f, 0.2f, 0.2f, 0.7f});
            }
            if (sess.active && rt.plan.valid) {
                double idx = (progress(this) - rt.plan.startProg) * 240.0 / std::max(1, g.tps);
                int from = std::max(0, (int)(idx / 12.0));
                ccColor4F col = rt.plan.full ? ccColor4F{0.2f, 1.f, 0.4f, 0.8f} : ccColor4F{1.f, 0.6f, 0.1f, 0.8f};
                for (int i = from; i < (int)rt.plan.path.size() && i < from + 70; i++) {
                    rt.draw->drawDot(rt.plan.path[i], 2.5f, col);
                }
            }
        }
    }
};
