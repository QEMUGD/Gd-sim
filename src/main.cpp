// Pathfinder - Geode mod for Geometry Dash 2.2081 (Windows + Android)
// Author: qemugd  https://github.com/qemugd/gd-sim
//
// Works like gdsolver (github.com/gdsolver/gdsolver, MIT): a planner proposes an input
// plan from a model, and the game itself verifies it by playing it.
//   1. Map the level (hazards, solid blocks, portals).
//   2. Plan a whole-level macro with a reachability search over 240 Hz ticks.
//   3. Replay the plan in the game, fast and dark, dumping the real state every tick.
//   4. When the game dies: find the first tick where the game and the model disagree and
//      record it as a fixup, re-anchor on the game's real state a few ticks before the
//      failure (a ladder of depths), plan the tail again and pin everything before it.
//      When the ladder runs out, search that section by perturbing inputs in the game.
//   5. When a plan clears, restart the level and show the solution at 1x, save it as a
//      solution file and as a .gdr2 replay (Eclipse Menu and other GDR2 bots).
//
// Credits
//   * Approach (planner proposes, the game verifies, repair by re-anchoring): gdsolver.
//   * Physics model: camila314's GD physics decompilations (camila314/gdp,
//     2.2 branch, used with permission). Constants and formulas below come from
//       - PlayerObject::updateTimeMod  (yStart / gravity / speed per speed portal)
//       - PlayerObject::updateJump     (jump rules for every vehicle, mini scaling,
//                                       gravity formula, fall clamps, robot boost)
//       - GJBaseGameLayer::update      (step count = delta * 240, physicsDelta)
//       - PlayLayer::destroyPlayer     (death handling, test-mode record block)
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
#include <sstream>
#include <chrono>
#include <deque>

using namespace geode::prelude;

// ==== SIM BEGIN
#include <unordered_map>
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

// one vehicle state; x/y are the player rect's minX/minY
struct SState {
    double x = 0, y = 0, w = 30, h = 30;
    double vy = 0, flip = 1, accel = 0, speed = 0.9;
    Mode mode = Mode::Cube;
    bool mini = false, grounded = true, down = false;
    int lastPress = -100000;
    int nextEvt = 0;
};

// A fixup is the model agreeing to be wrong: when the planner meets this exact
// (tick, state, input) again it applies what the game really did instead of its own result.
struct Fixup { SState after; bool dead; };

struct Env {
    std::vector<CCRect> const* hz = nullptr;    // sorted by minX
    std::vector<CCRect> const* sol = nullptr;   // sorted by minX
    std::vector<Evt> const* evts = nullptr;     // sorted by x
    std::unordered_map<uint64_t, Fixup> const* fix = nullptr;
    int    tickBase = 0;           // absolute tick of stepNo 0 (for fixup lookups)
    double floorBottom = 90.0;     // lowest allowed rect minY
    double ceilTop = 390.0;        // highest allowed rect maxY
    double gmod = 1.0;
    float  margin = 2.f;           // extra clearance around hazards
    int    lockSteps = 0;          // CPS lock: min 240 Hz steps between presses (0 = off)
    bool   allowHold = true;
    double xEnd = 1e9;
};

static uint64_t mix64(uint64_t h, uint64_t v) {
    h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
}
static uint64_t fixKey(int tick, SState const& s, bool act) {
    uint64_t k = 1469598103934665603ULL;
    k = mix64(k, (uint64_t)tick);
    k = mix64(k, (uint64_t)(int64_t)std::llround(s.y));
    k = mix64(k, (uint64_t)(int64_t)std::llround(s.vy * 2.0));
    k = mix64(k, (uint64_t)((s.flip > 0 ? 1 : 2) | ((int)s.mode << 2) | (s.grounded ? 64 : 0) | (act ? 128 : 0)));
    return k;
}

static float marginAt(Env const& e, double) { return e.margin; }

// Advances one 240 Hz physics step with the input `down` held. Returns false on death.
static bool stepSim(SState& s, bool down, Env const& e, int stepNo) {
    bool pressed = down && !s.down;
    if (pressed) s.lastPress = stepNo;
    s.down = down;

    if (e.fix) {   // the game already showed us what happens here
        auto it = e.fix->find(fixKey(e.tickBase + stepNo, s, down));
        if (it != e.fix->end()) {
            if (it->second.dead) return false;
            auto const& a = it->second.after;
            s.x = a.x; s.y = a.y; s.w = a.w; s.h = a.h; s.vy = a.vy; s.flip = a.flip; s.speed = a.speed;
            s.mode = a.mode; s.mini = a.mini; s.grounded = a.grounded; s.accel = 0;
            if (e.evts) {
                auto ev = std::lower_bound(e.evts->begin(), e.evts->end(), (float)(s.x + s.w * 0.5),
                    [](Evt const& q, float v) { return q.x < v; });
                s.nextEvt = (int)(ev - e.evts->begin());
            }
            return true;
        }
    }

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

// ------------------------------------------------------------------ edges
struct Edge { int tick; bool down; };
using Edges = std::vector<Edge>;

static Edges edgesFromBlocks(std::vector<uint8_t> const& blocks, int G, int tickBase, bool initialDown) {
    Edges out; bool prev = initialDown;
    for (size_t b = 0; b < blocks.size(); b++) {
        bool d = blocks[b] != 0;
        if (d != prev) { out.push_back({tickBase + (int)b * G, d}); prev = d; }
    }
    return out;
}
static bool downAtTick(Edges const& e, int tick) {
    bool d = false;
    for (auto const& x : e) { if (x.tick > tick) break; d = x.down; }
    return d;
}
static bool edgesValid(Edges const& e, int lockSteps) {
    int lastTick = -1000000, lastPress = -1000000; bool prev = false;
    for (auto const& x : e) {
        if (x.tick < 0 || x.tick <= lastTick || x.down == prev) return false;
        if (x.down) { if (lockSteps > 0 && x.tick - lastPress < lockSteps) return false; lastPress = x.tick; }
        lastTick = x.tick; prev = x.down;
    }
    return true;
}

// Runs a plan from `start` (absolute tick 0), tick by tick. trace[t] is the state at the start
// of tick t. Used to verify a plan in the model (and, in tests, as a stand-in game).
struct RunResult { bool cleared = false; int deathTick = -1; int ticks = 0; };
static RunResult runEdges(SState s, Env e, Edges const& edges, std::vector<SState>* trace, int maxTicks) {
    e.tickBase = 0;
    RunResult r; size_t ei = 0; bool cur = s.down;
    for (int t = 0; t < maxTicks; t++) {
        if (trace) trace->push_back(s);
        while (ei < edges.size() && edges[ei].tick <= t) cur = edges[ei++].down;
        if (!stepSim(s, cur, e, t)) { r.deathTick = t; r.ticks = t; return r; }
        if (s.x >= e.xEnd) { r.cleared = true; r.ticks = t + 1; return r; }
    }
    r.ticks = maxTicks;
    return r;
}

// first tick where the real run and the model disagree about the vehicle's state
static int findDivergence(std::vector<SState> const& pred, std::vector<SState> const& real, double yTol) {
    size_t n = std::min(pred.size(), real.size());
    for (size_t t = 0; t < n; t++) {
        if (std::abs(pred[t].y - real[t].y) > yTol || std::abs(pred[t].x - real[t].x) > 6.0 ||
            pred[t].mode != real[t].mode) return (int)t;
    }
    return -1;
}

// ------------------------------------------------------------------ repair loop
// The planner proposes, the game verifies. When the game dies: note where the model was
// wrong (fixups), re-anchor on the game's own state a few ticks earlier, plan the tail again
// and pin everything before the anchor. A ladder of anchor depths widens the retry; when the
// ladder runs out, the section is searched by perturbing the inputs in the real game.
struct Repair {
    Edges edges;
    std::unordered_map<uint64_t, Fixup> fix;
    struct Stuck { int tick; int k; };
    std::vector<Stuck> stuck;
    bool perturbing = false; int baseDeath = 0; std::vector<Edges> variants; size_t vi = 0;
    int repairs = 0, anchorTick = 0, anchorDepth = 0, perturbCount = 0;
    std::string note;
};
enum class RepairResult { Replan, Perturb, GiveUp };

static std::vector<Edges> genVariants(Edges const& base, int deathTick, int lockSteps) {
    std::vector<Edges> out;
    int w0 = std::max(0, deathTick - 420);
    static const int shifts[] = {-8, -6, -4, -3, -2, -1, 1, 2, 3, 4, 6, 8};
    for (size_t i = 0; i < base.size(); i++) {
        if (base[i].tick < w0 || base[i].tick > deathTick) continue;
        for (int d : shifts) {                                 // move one edge
            Edges v = base; v[i].tick += d;
            if (edgesValid(v, lockSteps)) out.push_back(std::move(v));
        }
        if (base[i].down && i + 1 < base.size()) {            // move a whole press (press + release)
            for (int d : shifts) {
                Edges v = base; v[i].tick += d; v[i + 1].tick += d;
                if (edgesValid(v, lockSteps)) out.push_back(std::move(v));
            }
        }
    }
    for (int t = std::max(8, w0); t < deathTick - 4; t += 6) {   // add a tap
        Edges v = base;
        if (downAtTick(base, t) || downAtTick(base, t + 2)) continue;
        bool clash = false; for (auto const& x : base) if (x.tick >= t - 1 && x.tick <= t + 3) clash = true;
        if (clash) continue;
        v.push_back({t, true}); v.push_back({t + 2, false});
        std::sort(v.begin(), v.end(), [](Edge const& a, Edge const& b) { return a.tick < b.tick; });
        if (edgesValid(v, lockSteps)) out.push_back(std::move(v));
    }
    // the smallest changes first
    auto cost = [&](Edges const& v) {
        long c = 0;
        for (size_t i = 0; i < v.size() && i < base.size(); i++) c += std::abs(v[i].tick - base[i].tick);
        c += 20L * std::abs((long)v.size() - (long)base.size());
        return c;
    };
    std::stable_sort(out.begin(), out.end(), [&](Edges const& a, Edges const& b) { return cost(a) < cost(b); });
    if (out.size() > 400) out.resize(400);
    return out;
}

static RepairResult repairAfterDeath(Repair& r, Env planEnv, SState const& s0,
                                     std::vector<SState> const& pred, std::vector<SState> const& real,
                                     int deathTick, int G, int beam, double yTol) {
    static const int ladder[] = {6, 24, 72, 200, 500, 1200};
    const int nLadder = 6;
    r.repairs++;

    // 1. section search: was the variant we just tried good enough?
    if (r.perturbing) {
        r.perturbCount++;
        if (deathTick > r.baseDeath + 12) {
            r.edges = r.variants[r.vi];
            r.perturbing = false; r.variants.clear();
            r.note = "section search found a variant that gets further; continuing";
        } else {
            r.vi++;
            if (r.vi < r.variants.size()) { r.edges = r.variants[r.vi]; return RepairResult::Perturb; }
            r.perturbing = false;
            r.note = "section search exhausted";
            return RepairResult::GiveUp;
        }
    }

    // 2. record where the model was wrong
    int d = findDivergence(pred, real, yTol);
    if (d > 0 && (size_t)d < real.size() && (size_t)(d - 1) < pred.size()) {
        bool act = downAtTick(r.edges, d - 1);
        Fixup f; f.after = real[d]; f.dead = false;
        r.fix[fixKey(d - 1, pred[d - 1], act)] = f;
    }
    {   // the death itself: the model thought this input was safe, the game did not
        int dt = (int)real.size() - 1;
        if (dt >= 0) {
            bool act = downAtTick(r.edges, dt);
            Fixup f; f.dead = true;
            r.fix[fixKey(dt, real[dt], act)] = f;
            if ((size_t)dt < pred.size()) r.fix[fixKey(dt, pred[dt], act)] = f;
        }
    }

    // 3. how stuck are we here? walk down the ladder of anchor depths
    Repair::Stuck* st = nullptr;
    for (auto& x : r.stuck) if (std::abs(x.tick - deathTick) < 240) { st = &x; break; }
    if (!st) { r.stuck.push_back({deathTick, 0}); st = &r.stuck.back(); } else st->k++;

    int base = (d >= 0) ? std::min(d, deathTick) : deathTick;
    for (int k = st->k; k < nLadder; k++) {
        int anchor = std::max(0, base - ladder[k]);
        SState sa = (anchor == 0) ? s0 : real[std::min((size_t)anchor, real.size() - 1)];
        Edges prefix; for (auto const& x : r.edges) if (x.tick < anchor) prefix.push_back(x);
        bool downA = downAtTick(prefix, anchor);
        int lastPressTick = -100000; for (auto const& x : prefix) if (x.down) lastPressTick = x.tick;
        sa.down = downA;
        sa.lastPress = lastPressTick < -50000 ? -100000 : lastPressTick - anchor;

        Env e = planEnv; e.fix = &r.fix; e.tickBase = anchor;
        auto evIt = std::lower_bound(e.evts->begin(), e.evts->end(), (float)(sa.x + sa.w * 0.5),
            [](Evt const& q, float v) { return q.x < v; });
        sa.nextEvt = (int)(evIt - e.evts->begin());
        int maxLayers = (int)(std::max(0.0, e.xEnd + 120.0 - sa.x) / (0.7 * 5.980002 * 60.0 / 240.0) / G) + 400;
        PlanOut out = planMacro(sa, e, G, beam, r.repairs + k, maxLayers);
        if (!out.reachedEnd) { st->k = k + 1; continue; }

        Edges tail = edgesFromBlocks(out.down, G, anchor, downA);
        Edges full = prefix; full.insert(full.end(), tail.begin(), tail.end());
        {   // the same plan again would just die the same way: go deeper instead
            bool same = full.size() == r.edges.size();
            for (size_t q = 0; same && q < full.size(); q++)
                same = full[q].tick == r.edges[q].tick && full[q].down == r.edges[q].down;
            if (same) { st->k = k + 1; continue; }
        }
        // verify in the model (with the fixups) from the very start before handing it to the game
        Env ve = planEnv; ve.fix = &r.fix; ve.margin = 0.f;
        RunResult rr = runEdges(s0, ve, full, nullptr, 400000);
        if (!rr.cleared) { st->k = k + 1; continue; }
        r.edges = full; r.anchorTick = anchor; r.anchorDepth = ladder[k];
        r.note = "re-anchored " + std::to_string(ladder[k]) + " ticks before the failure";
        st->k = k;
        return RepairResult::Replan;
    }

    // 4. the model cannot fix it: search this section with the real game
    r.variants = genVariants(r.edges, deathTick, planEnv.lockSteps);
    r.baseDeath = deathTick; r.vi = 0;
    if (r.variants.empty()) { r.note = "no variants to try"; return RepairResult::GiveUp; }
    r.perturbing = true;
    r.edges = r.variants[0];
    r.note = "model cannot clear this; searching the section in the game (" + std::to_string(r.variants.size()) + " variants)";
    return RepairResult::Perturb;
}
// ==== SIM END

// ================================================================= state
static constexpr int kG = 2;   // the planner chooses hold/release once per kG physics ticks (240 Hz)

struct PFState {
    // ---- settings (saved)
    int   mode         = 0;       // 0 = solve, 1 = replay the stored solution
    bool  record       = true;
    bool  autoSave     = true;
    bool  overlay      = true;
    bool  allowHold    = true;    // allow held inputs (otherwise taps only)
    bool  dark         = true;    // fast dark loop: hide the level while solving
    int   beam         = 256;     // planner width (states kept per tick)
    float margin       = 1.f;     // extra clearance around hazards for the first plan
    float speed        = 10.f;    // game speed while solving
    float watchSpeed   = 1.f;     // game speed while showing / replaying the solution
    int   tps          = 240;     // physics ticks per second (240 = vanilla)
    bool  cpsLock      = true;    // limit presses per second
    int   cps          = 16;
    int   maxAttempts  = 600;

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

enum class Phase { Idle, Planning, Solving, Showing, Replaying, Passed, GaveUp };
struct Session {
    bool  armed = false, active = false;
    Phase phase = Phase::Idle;
    int   attempt = 0;
    float best = 0.f;
    int   bestTick = 0;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
};
static Session sess;

struct LevelMap {
    std::vector<CCRect> hz, sol;   // sorted by minX
    std::vector<Evt> evts;         // sorted by x
    float xEnd = 0;
    int unsupported = 0;           // orbs / pads / dual / mirror we do not model
};
static LevelMap lmap;

struct Runtime {
    std::vector<std::pair<uint64_t, bool>> inputs; // (frame, down) as recorded
    std::vector<SState> trace, pred;               // real state per tick / the model's prediction
    SState s0;                                     // the real state at tick 0
    Edges solution;
    bool   held = false;                           // real P1 input state
    int    lastPressTick = -100000;
    int    lastProg = -1;
    int    startProg = 0;
    int    lastTickSeen = -1;
    size_t edgeIdx = 0;
    bool   pendingShow = false;
    bool   needPlan = false;                       // plan from the real tick-0 state
    bool   deathHandled = false, completed = false;
    bool   cmdDown = false;                        // what we last told the game
    int    injectMode = 0;                         // 0 = handleButton, 1 = pushButton/releaseButton
    int    noEffect = 0; bool inputOk = false, inputChecked = true;
    int    pressStep = -1; double pressY = 0; bool pressGrounded = false; Mode pressMode = Mode::Cube;
    std::string hudNote;
    double floorBottom = 90.0;                     // level bounds (game coords)
    double ceilTop     = 390.0;
    double gmod = 1.0;
    double accum = 0.0;                            // TPS bypass time accumulator
    CCDrawNode* draw = nullptr;
    CCLabelBMFont* label = nullptr;
};
static Runtime rt;
static Repair rep;

static int progress(GJBaseGameLayer* l) { return (int)l->m_gameState.m_currentProgress; }
static int simTick(int prog) { return (int)((prog - rt.startProg) * 240.0 / std::max(1, g.tps)); }

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
    m->setSavedValue("mode", g.mode);         m->setSavedValue("record", g.record);
    m->setSavedValue("autosave", g.autoSave); m->setSavedValue("overlay", g.overlay);
    m->setSavedValue("allowhold", g.allowHold); m->setSavedValue("dark", g.dark);
    m->setSavedValue("beam", g.beam);         m->setSavedValue("margin", g.margin);
    m->setSavedValue("speed", g.speed);       m->setSavedValue("watchspeed", g.watchSpeed);
    m->setSavedValue("tps", g.tps);           m->setSavedValue("cpslock", g.cpsLock);
    m->setSavedValue("cps", g.cps);           m->setSavedValue("maxattempts", g.maxAttempts);
}
static void loadSettings() {
    auto m = Mod::get();
    g.mode = m->getSavedValue<int>("mode", 0);
    g.record = m->getSavedValue<bool>("record", true);
    g.autoSave = m->getSavedValue<bool>("autosave", true);
    g.overlay = m->getSavedValue<bool>("overlay", true);
    g.allowHold = m->getSavedValue<bool>("allowhold", true);
    g.dark = m->getSavedValue<bool>("dark", true);
    g.beam = m->getSavedValue<int>("beam", 256);
    g.margin = m->getSavedValue<float>("margin", 1.f);
    g.speed = m->getSavedValue<float>("speed", 10.f);
    g.watchSpeed = m->getSavedValue<float>("watchspeed", 1.f);
    g.tps = m->getSavedValue<int>("tps", 240);
    g.cpsLock = m->getSavedValue<bool>("cpslock", true);
    g.cps = m->getSavedValue<int>("cps", 16);
    g.maxAttempts = m->getSavedValue<int>("maxattempts", 600);
    g.tps = std::clamp(g.tps, 240, 5000);
    g.speed = std::clamp(g.speed, 1.f, 10.f);
    g.watchSpeed = std::clamp(g.watchSpeed, 0.25f, 4.f);
    g.beam = std::clamp(g.beam, 32, 1024);
    g.mode = std::clamp(g.mode, 0, 1);
}

static void setTimeScale(float v) {
    if (auto d = CCDirector::get()) {
        if (auto sch = d->getScheduler()) sch->setTimeScale(v);
    }
}

// ================================================================= .gdr2 export + solution files
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

    PFReplay rep2;
    rep2.author      = std::string(GJAccountManager::get()->m_username.c_str());
    rep2.description = "Generated by Pathfinder";
    rep2.gameVersion = 22081;
    rep2.framerate   = (double)g.tps;          // physics ticks per second the run was played at
    rep2.seed        = 0;
    rep2.coins       = 0;
    rep2.ldm         = false;
    rep2.platformer  = pl->m_level->isPlatformer();
    rep2.levelInfo.id   = pl->m_level->m_levelID.value();
    rep2.levelInfo.name = std::string(pl->m_level->m_levelName.c_str());

    uint64_t last = 0;
    for (auto const& [frame, down] : rt.inputs) {
        rep2.inputs.push_back(gdr::Input<>(frame, 1, false, down));
        last = frame;
    }
    rep2.duration = (float)(last / (double)g.tps);

    auto bytes = unwrapBytes(rep2.exportData());
    if (!bytes) { g.status = "Export failed (GDR could not serialize the replay)."; return false; }

    std::string name = std::string(pl->m_level->m_levelName.c_str());
    for (auto& c : name) if (!std::isalnum((unsigned char)c)) c = '_';
    auto stamp = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    auto path = replayDir() / fmt::format("{}_{}_{}.gdr2", name, (int)pl->m_level->m_levelID.value(), stamp);

    auto res = geode::utils::file::writeBinary(path, *bytes);
    if (!res) { g.status = "Could not write file: " + path.string(); return false; }

    g.status = "Saved " + path.filename().string() + (g.dirty ? "  (had practice respawns)" : "");
    plog(fmt::format("Saved replay {} ({} inputs, {} TPS)", path.filename().string(), rep2.inputs.size(), g.tps));
    return true;
}

// solution file: one `input=<tick>,<0|1>` line per press or release (240 Hz tick)
static std::filesystem::path solutionPath(PlayLayer* pl) {
    return Mod::get()->getSaveDir() / fmt::format("solution_{}.txt", (int)pl->m_level->m_levelID.value());
}
static void saveSolution(PlayLayer* pl, Edges const& edges) {
    std::ofstream f(solutionPath(pl));
    if (!f) { plog("Could not write the solution file."); return; }
    f << "# Pathfinder solution for level " << (int)pl->m_level->m_levelID.value() << "\n";
    for (auto const& e : edges) f << "input=" << e.tick << "," << (e.down ? 1 : 0) << "\n";
    plog(fmt::format("Solution written to {}", solutionPath(pl).filename().string()));
}
static bool loadSolution(PlayLayer* pl, Edges& out) {
    std::ifstream f(solutionPath(pl));
    if (!f) return false;
    out.clear();
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("input=", 0) != 0) continue;
        int tick = 0, d = 0;
        if (std::sscanf(line.c_str() + 6, "%d,%d", &tick, &d) == 2) out.push_back({tick, d != 0});
    }
    return !out.empty();
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

// Scans every object once, at the level's start: static hitboxes of hazards and solid
// blocks, plus portals. (Not repeated mid-run: moving objects would corrupt the map.)
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

// ================================================================= model <-> game
static void learnBounds(PlayerObject* p, Mode m) {
    // Only trust surface contact in modes that live between floor and ceiling.
    if (m == Mode::Cube || m == Mode::Robot || m == Mode::Spider) return;
    if (!p->m_isOnGround) return;
    auto r = p->getObjectRect();
    if (p->m_isUpsideDown) rt.ceilTop = r.getMaxY();
    else                   rt.floorBottom = r.getMinY();
}

// the game's own state as a model state
static SState realState(PlayerObject* p) {
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
    s.lastPress = rt.lastPressTick;
    auto it = std::lower_bound(lmap.evts.begin(), lmap.evts.end(), (float)(s.x + s.w * 0.5),
        [](Evt const& e, float v) { return e.x < v; });
    s.nextEvt = (int)(it - lmap.evts.begin());
    return s;
}

static Env makeEnv(float marginScale) {
    Env e;
    e.hz = &lmap.hz; e.sol = &lmap.sol; e.evts = &lmap.evts; e.fix = &rep.fix;
    e.floorBottom = rt.floorBottom; e.ceilTop = rt.ceilTop;
    e.gmod = rt.gmod;
    e.margin = g.margin * marginScale;
    e.lockSteps = (g.cpsLock && g.cps > 0) ? (int)std::lround(240.0 / g.cps) : 0;
    e.allowHold = g.allowHold;
    e.xEnd = lmap.xEnd;
    return e;
}

static void recomputePred() {
    rt.pred.clear();
    runEdges(rt.s0, makeEnv(0.f), rep.edges, &rt.pred, 400000);   // the model's view of the plan
}

static int maxLayersFrom(double x) {
    double minV = 0.7 * 5.980002 * 60.0 / 240.0;   // slowest speed, units per tick
    return (int)(std::max(0.0, lmap.xEnd + 120.0 - x) / minV / kG) + 400;
}

// ================================================================= session
static float elapsedSeconds() {
    return std::chrono::duration<float>(std::chrono::steady_clock::now() - sess.t0).count();
}

static void endSession(std::string const& why, Phase result) {
    if (!sess.active) return;
    sess.active = false;
    sess.phase = result;
    setTimeScale(1.f);
    plog(why);
}

// first plan from the game's real start state: plan -> verify -> retry with other margins
static void planInitial(PlayLayer* pl) {
    sess.phase = Phase::Planning;
    auto p = pl->m_player1;
    rt.gmod = p->m_gravityMod;
    rt.s0 = realState(p);
    rt.s0.down = false; rt.s0.lastPress = -100000;
    rep = Repair{};
    plog(fmt::format("Start state (real, tick 0): x={:.1f} y={:.1f} size {:.0f}x{:.0f} vy={:.2f} {} speed {:.2f} grounded {}",
        rt.s0.x, rt.s0.y, rt.s0.w, rt.s0.h, rt.s0.vy, modeName(rt.s0.mode), rt.s0.speed, rt.s0.grounded ? "yes" : "no"));
    if (!lmap.hz.empty()) {
        auto const& h0 = lmap.hz.front();
        plog(fmt::format("First hazard hitbox: x={:.1f} y={:.1f} {:.0f}x{:.0f}", h0.getMinX(), h0.getMinY(), h0.size.width, h0.size.height));
    } else {
        plog("WARNING: no hazards were mapped. The planner will think the level is empty.");
        rt.hudNote = "WARNING: no hazards mapped";
    }

    float scales[3] = {1.f, 0.5f, 0.f};
    PlanOut bestPartial; bool havePartial = false; bool ok = false;
    for (int t = 0; t < 3 && !ok; t++) {
        Env env = makeEnv(scales[t]);
        auto t0 = std::chrono::steady_clock::now();
        PlanOut out = planMacro(rt.s0, env, kG, g.beam, t, maxLayersFrom(rt.s0.x));
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        if (out.reachedEnd) {
            Edges ed = edgesFromBlocks(out.down, kG, 0, false);
            RunResult rr = runEdges(rt.s0, makeEnv(0.f), ed, nullptr, 400000);
            if (rr.cleared) {
                rep.edges = ed; ok = true;
                int presses = 0; for (auto const& e : ed) if (e.down) presses++;
                g.planInfo = fmt::format("First plan: {} presses, planned in {:.0f} ms (margin {:.1f}). Waiting for the game to verify it.",
                                         presses, ms, env.margin);
                plog(fmt::format("[plan] try {} (margin {:.1f}): reached the end in {:.0f} ms | model check PASSED | {} presses",
                                 t + 1, env.margin, ms, presses));
            } else {
                plog(fmt::format("[plan] try {} (margin {:.1f}): model check FAILED at tick {} - retrying", t + 1, env.margin, rr.deathTick));
            }
        } else {
            plog(fmt::format("[plan] try {} (margin {:.1f}): no safe path past x={:.0f} (level end {:.0f}) - retrying",
                             t + 1, env.margin, out.reachX, lmap.xEnd));
            if (!havePartial || out.reachX > bestPartial.reachX) { bestPartial = std::move(out); havePartial = true; }
        }
    }
    if (!ok) {
        if (havePartial) rep.edges = edgesFromBlocks(bestPartial.down, kG, 0, false);
        g.planInfo = fmt::format("No complete plan from the model. Starting from the best partial plan (to x={:.0f}); the game will repair it.",
                                 havePartial ? bestPartial.reachX : 0.0);
        plog("[plan] no complete plan; the repair loop will work from the best partial plan.");
    }
    {
        std::string first;
        int n = 0;
        for (auto const& e : rep.edges) { if (n++ >= 8) break; first += fmt::format(" {}{}", e.down ? "press@" : "release@", e.tick); }
        plog(fmt::format("Plan has {} inputs; first:{}", rep.edges.size(), first.empty() ? std::string(" (none)") : first));
    }
    recomputePred();
    sess.phase = Phase::Solving;
}

static void beginSession(PlayLayer* pl) {
    sess.armed = false;
    sess.active = true;
    sess.attempt = 0; sess.best = 0.f; sess.bestTick = 0;
    sess.t0 = std::chrono::steady_clock::now();
    rt.solution.clear();
    plog(fmt::format("=== Pathfinder: {} (ID {}) | {} | {:.1f}x, {} TPS, CPS lock {}, holding {} ===",
        std::string(pl->m_level->m_levelName.c_str()), (int)pl->m_level->m_levelID.value(),
        g.mode == 1 ? "replay stored solution" : "solve",
        g.speed, g.tps, g.cpsLock ? std::to_string(g.cps) : std::string("off"), g.allowHold ? "on" : "off"));
    plog("Records are protected while the bot drives (the level is played in test mode).");

    mapLevel(pl);
    plog(fmt::format("Mapped level: {} hazards, {} solid blocks, {} portals, end at x={:.0f}.",
        lmap.hz.size(), lmap.sol.size(), lmap.evts.size(), lmap.xEnd));
    if (lmap.unsupported > 0) {
        plog(fmt::format("Warning: {} orbs/pads/dual/mirror objects are not modelled; the game will have to repair around them.",
            lmap.unsupported));
    }
    auto p = pl->m_player1;
    rt.gmod = p ? p->m_gravityMod : 1.0;
    if (p) { rt.s0 = realState(p); rt.s0.down = false; rt.s0.lastPress = -100000; }

    if (g.mode == 1) {                                   // replay the stored solution
        Edges e;
        if (!loadSolution(pl, e)) {
            plog("No stored solution for this level. Solve it first.");
            endSession("Nothing to replay.", Phase::Idle);
            return;
        }
        rep = Repair{}; rep.edges = e;
        sess.phase = Phase::Replaying;
        setTimeScale(g.watchSpeed);
        plog(fmt::format("Replaying the stored solution ({} inputs) at {:.2f}x.", e.size(), g.watchSpeed));
        return;
    }
    rt.needPlan = true;                 // plan from the real state at tick 0, not from the loading screen
    rt.hudNote = "";
    sess.phase = Phase::Planning;
}

// the game died: the plan was wrong somewhere. Repair it from the game's own state.
static void onDeath(PlayLayer* pl, bool fromReset) {
    rt.deathHandled = true;
    int deathTick = std::max(0, (int)rt.trace.size() - 1);
    float x = rt.trace.empty() ? 0.f : (float)rt.trace.back().x;
    float pct = (!fromReset) ? pl->getCurrentPercent()
                             : (lmap.xEnd > 0 ? std::clamp(x / lmap.xEnd * 100.f, 0.f, 100.f) : 0.f);
    sess.attempt++;
    if (pct > sess.best) sess.best = pct;
    sess.bestTick = std::max(sess.bestTick, deathTick);

    if (sess.phase == Phase::Replaying) {
        endSession(fmt::format("The stored solution died at {:.1f}% (x={:.0f}); it does not clear on this setup.", pct, x), Phase::Idle);
        return;
    }
    if (sess.phase == Phase::Showing) {
        plog(fmt::format("The solution did not replay cleanly at 1x (died at {:.1f}%). Back to solving.", pct));
        sess.phase = Phase::Solving;
        setTimeScale(g.speed);
    }

    // the game's real tick-0 state is the truth: if the plan started from a different one, fix that first
    if (!rt.trace.empty()) {
        SState r0 = rt.trace[0]; r0.down = false; r0.lastPress = -100000;
        if (std::abs(r0.x - rt.s0.x) > 0.5 || std::abs(r0.y - rt.s0.y) > 0.5 || r0.mode != rt.s0.mode) {
            plog(fmt::format("The plan started from x={:.1f} y={:.1f} but the game started at x={:.1f} y={:.1f}; using the game's.",
                rt.s0.x, rt.s0.y, r0.x, r0.y));
            rt.s0 = r0;
            recomputePred();
        }
    }

    int div = findDivergence(rt.pred, rt.trace, 5.0);
    std::string killer = (!rt.trace.empty()) ? modeName(rt.trace.back().mode) : "?";
    plog(fmt::format("[attempt {}] DIED at {:.1f}% (tick {}, x={:.0f}, {}) | first model/game disagreement: {}",
        sess.attempt, pct, deathTick, x, killer,
        div >= 0 ? fmt::format("tick {}", div) : std::string("none (the model was blind to this)")));
    if (div >= 0 && (size_t)div < rt.trace.size() && (size_t)div < rt.pred.size()) {
        plog(fmt::format("    at tick {}: game y={:.1f} x={:.1f} | model y={:.1f} x={:.1f}",
            div, rt.trace[div].y, rt.trace[div].x, rt.pred[div].y, rt.pred[div].x));
    }

    auto t0 = std::chrono::steady_clock::now();
    RepairResult res = repairAfterDeath(rep, makeEnv(1.f), rt.s0, rt.pred, rt.trace, deathTick, kG, g.beam, 5.0);
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    plog(fmt::format("    repair {} ({:.0f} ms, {} fixups): {}", rep.repairs, ms, rep.fix.size(), rep.note));
    g.planInfo = fmt::format("Repair {}: {} ({} fixups).", rep.repairs, rep.note, rep.fix.size());
    if (rt.hudNote.rfind("WARNING", 0) != 0) rt.hudNote = rep.note;

    if (res == RepairResult::GiveUp) {
        endSession(fmt::format("Gave up at {:.1f}% after {} attempts: {}.", sess.best, sess.attempt, rep.note), Phase::GaveUp);
    } else if (sess.attempt >= g.maxAttempts) {
        endSession(fmt::format("Reached the attempt limit ({}). Best {:.1f}%.", g.maxAttempts, sess.best), Phase::GaveUp);
    } else {
        recomputePred();
    }
}

static void applyInput(PlayLayer* pl, bool want) {
    if (rt.injectMode == 0) pl->handleButton(want, 1, true);
    else if (want) pl->pushButton(1, true);
    else pl->releaseButton(1, true);
    rt.cmdDown = want;
}

// did a press really reach the game? If not, say so loudly and try the other input method.
static void checkInputEffect(PlayLayer* pl, PlayerObject* p, int step) {
    if (rt.inputChecked || rt.pressStep < 0 || step < rt.pressStep + 3) return;
    rt.inputChecked = true;
    bool groundMode2 = rt.pressMode == Mode::Cube || rt.pressMode == Mode::Robot ||
                       rt.pressMode == Mode::Ball || rt.pressMode == Mode::Spider;
    if (!rt.pressGrounded || !groundMode2) return;
    double y = p->getObjectRect().getMinY();
    bool moved = std::abs(y - rt.pressY) > 1.0 || !p->m_isOnGround;
    if (moved) {
        rt.noEffect = 0;
        if (!rt.inputOk) { rt.inputOk = true; plog(fmt::format("Input check OK: the press at tick {} moved the player.", rt.pressStep)); }
        return;
    }
    rt.noEffect++;
    plog(fmt::format("Input check FAILED: the press at tick {} did not move the player ({} in a row).", rt.pressStep, rt.noEffect));
    rt.hudNote = "WARNING: presses are not reaching the game";
    if (rt.noEffect >= 2 && rt.injectMode == 0) {
        rt.injectMode = 1; rt.noEffect = 0;
        plog("Switching the input method to pushButton/releaseButton.");
        rt.hudNote = "input method switched";
    } else if (rt.noEffect >= 3 && rt.injectMode == 1) {
        plog("Neither input method moves the player. Another mod may be blocking input.");
        rt.hudNote = "no input method works - check other mods";
    }
}

static void startAttempt() {
    rt.trace.clear();
    rt.edgeIdx = 0;
    rt.lastTickSeen = -1;
    rt.lastPressTick = -100000;
    rt.held = false;
    rt.cmdDown = false;
    rt.deathHandled = false;
    rt.completed = false;
    rt.pressStep = -1;
    rt.inputChecked = true;
}

// Runs once per physics step, right before the game processes input.
static void tick(PlayLayer* pl) {
    int prog = progress(pl);
    if (prog == rt.lastProg) return;            // one decision per physics step
    rt.lastProg = prog;

    auto p = pl->m_player1;
    if (!p || p->m_isDead || !sess.active) return;
    learnBounds(p, modeOf(p));

    int step = simTick(prog);
    if (step < 0) return;

    // first tick of the first attempt: plan from the game's real start state
    if (rt.needPlan) {
        rt.needPlan = false;
        planInitial(pl);
        setTimeScale(sess.phase == Phase::Replaying ? g.watchSpeed : g.speed);
    }

    // dump the game's real state at the start of every 240 Hz tick
    while ((int)rt.trace.size() <= step) rt.trace.push_back(realState(p));
    checkInputEffect(pl, p, step);

    // apply the plan
    while (rt.edgeIdx < rep.edges.size() && rep.edges[rt.edgeIdx].tick <= step) {
        bool want = rep.edges[rt.edgeIdx++].down;
        if (want != rt.cmdDown) {
            if (want && rt.inputChecked) {          // remember the first press so we can check it worked
                rt.pressStep = step; rt.pressY = p->getObjectRect().getMinY();
                rt.pressGrounded = p->m_isOnGround; rt.pressMode = modeOf(p); rt.inputChecked = false;
            }
            applyInput(pl, want);
            if (want) g.clicks++;
        }
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

static std::string barStr(float frac, int n = 20) {
    frac = std::clamp(frac, 0.f, 1.f);
    int f = (int)std::lround(frac * n);
    return std::string(f, '#') + std::string(n - f, '.');
}

static char const* phaseName(Phase p) {
    switch (p) {
        case Phase::Idle: return "Idle";               case Phase::Planning: return "Planning";
        case Phase::Solving: return "Solving (fast loop)"; case Phase::Showing: return "Showing the solution";
        case Phase::Replaying: return "Replaying";     case Phase::Passed: return "Solved";
        case Phase::GaveUp: return "Gave up";
    }
    return "?";
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
    ImGui::SeparatorText("Mode");
    ImGui::RadioButton("Solve the level", &g.mode, 0);
    ImGui::RadioButton("Replay the stored solution", &g.mode, 1);
    ImGui::SeparatorText("Speed");
    ImGui::SliderFloat("Solving speed", &g.speed, 1.f, 10.f, "%.1fx");
    ImGui::SliderFloat("Watching speed", &g.watchSpeed, 0.25f, 4.f, "%.2fx");
    ImGui::Checkbox("Fast dark loop (hide the level while solving)", &g.dark);
    ImGui::SliderInt("TPS", &g.tps, 240, 5000, "%d", ImGuiSliderFlags_Logarithmic);
    ImGui::TextDisabled("240 = vanilla physics. Higher TPS is experimental and heavy.");
    ImGui::SeparatorText("Clicks");
    ImGui::Checkbox("CPS lock", &g.cpsLock);
    ImGui::BeginDisabled(!g.cpsLock);
    ImGui::SliderInt("Clicks per second", &g.cps, 1, 60);
    ImGui::EndDisabled();
    ImGui::Checkbox("Allow holding", &g.allowHold);
    ImGui::TextDisabled("Holds are used for ship, wave, robot and cube re-jumps. Off = taps only.");
    ImGui::SeparatorText("Solver");
    ImGui::SliderInt("Max attempts", &g.maxAttempts, 1, 3000);
    ImGui::SliderInt("Planner width", &g.beam, 32, 1024);
    ImGui::SliderFloat("First-plan margin", &g.margin, 0.f, 8.f, "%.1f");
    ImGui::SeparatorText("Output");
    ImGui::Checkbox("Record inputs (.gdr2)", &g.record);
    ImGui::Checkbox("Auto-save when solved", &g.autoSave);
    ImGui::Checkbox("Show hazards + planned path", &g.overlay);
}

static void drawSetupWindow() {
    auto ds = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos({ds.x * 0.5f, ds.y * 0.5f}, ImGuiCond_Always, {0.5f, 0.5f});
    ImGui::SetNextWindowSize({std::min(460.f, ds.x * 0.92f), std::min(640.f, ds.y * 0.92f)}, ImGuiCond_Always);
    ImGui::Begin("Pathfinder setup", nullptr,
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoTitleBar);
    drawHeader("PATHFINDER", "Plan it, let the game verify it, repair it");

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
    if (ImGui::Button(g.mode == 1 ? "OK  -  Replay solution" : "OK  -  Start solving", {half, 0})) startPathfind();
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
                ImGui::Text("Attempt: %d   Best: %.1f%%   Repairs: %d", sess.attempt, sess.best, rep.repairs);
                ImGui::ProgressBar(std::clamp(g.percent / 100.f, 0.f, 1.f), {-1, 0},
                    fmt::format("{:.1f}%", g.percent).c_str());
                ImGui::TextWrapped("%s", g.planInfo.c_str());
                ImGui::Text("Fixups: %d   Anchor depth: %d ticks   Section tries: %d",
                    (int)rep.fix.size(), rep.anchorDepth, rep.perturbCount);
                ImGui::Text("Inputs recorded: %d  (presses: %d)", (int)rt.inputs.size(), g.clicks);
                if (g.dirty) ImGui::TextColored({1.f, 0.7f, 0.2f, 1.f}, "Run has practice respawns");
                if (ImGui::Button("Save .gdr2 now", {-1, 0})) {
                    if (auto pl = PlayLayer::get()) saveReplay(pl);
                }
                if (ImGui::Button("Clear inputs", {-1, 0})) { rt.inputs.clear(); rt.held = false; g.status = "Cleared."; }
            } else {
                ImGui::TextWrapped("Open a level page and press the cube button to solve it.");
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
            ImGui::SeparatorText("Repairs");
            ImGui::Text("Fixups (where the model was wrong): %d", (int)rep.fix.size());
            if (rep.stuck.empty()) ImGui::TextDisabled("no failure spots yet");
            for (auto const& z : rep.stuck) ImGui::BulletText("tick %d  ladder step %d", z.tick, z.k);
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
        if (down) rt.lastPressTick = simTick(progress(this));

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
        rt.trace.clear(); rt.pred.clear();
        rt.held = false;
        rt.lastPressTick = -100000;
        rt.lastProg = -1;
        rt.startProg = 0;
        rt.edgeIdx = 0;
        rt.pendingShow = false;
        rt.accum = 0.0;
        rt.floorBottom = 90.0;
        rt.ceilTop = 390.0;
        g.planInfo = "No plan yet.";

        rt.draw = CCDrawNode::create();
        m_objectLayer->addChild(rt.draw, 1000);

        auto winSize = CCDirector::get()->getWinSize();
        rt.label = CCLabelBMFont::create("", "bigFont.fnt");
        rt.label->setScale(0.34f);
        rt.label->setAnchorPoint({0.f, 1.f});
        rt.label->setPosition({8.f, winSize.height - 8.f});
        rt.label->setVisible(false);
        this->addChild(rt.label, 200);

        if (sess.armed) {
            m_isTestMode = true;               // the game's own "do not record" path
            beginSession(this);
        } else {
            mapLevel(this);
            g.status = "Level loaded.";
        }
    }

    void resetLevel() {
        PlayLayer::resetLevel();
        if (sess.active) m_isTestMode = true;
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
        // a restart without a recorded death or a clear still means the attempt failed
        if (sess.active && !rt.deathHandled && !rt.completed && !rt.needPlan && rt.trace.size() > 5 &&
            (sess.phase == Phase::Solving || sess.phase == Phase::Showing || sess.phase == Phase::Replaying)) {
            plog("(the level restarted without a death event; treating it as a failed attempt)");
            onDeath(this, true);
        }
        rt.startProg = progress(this);
        rt.lastProg = -1;
        if (sess.active) startAttempt();
    }

    void destroyPlayer(PlayerObject* player, GameObject* obj) {
        bool wasDead = m_playerDied || (player && player->m_isDead);
        PlayLayer::destroyPlayer(player, obj);
        bool died = m_playerDied || (player && player->m_isDead);
        if (wasDead || !died || !sess.active || player != m_player1) return;   // anti-cheat spike etc. don't count
        if (obj) {
            auto pos = obj->getPosition();
            plog(fmt::format("    killed by object id {} at ({:.0f}, {:.0f})", (int)obj->m_objectID, pos.x, pos.y));
        }
        onDeath(this, false);
    }

    void levelComplete() {
        if (sess.active && sess.phase == Phase::Solving) {
            // the game cleared the plan: that is a solution. Show it properly at 1x.
            if (rt.pendingShow) return;
            sess.attempt++;
            sess.best = 100.f;
            rt.solution = rep.edges;
            plog(fmt::format("CLEARED in the fast loop on attempt {} after {:.0f}s. Restarting to show it at 1x.",
                             sess.attempt, elapsedSeconds()));
            saveSolution(this, rt.solution);
            rt.pendingShow = true;
            rt.completed = true;
            return;
        }
        PlayLayer::levelComplete();
        if (sess.active && (sess.phase == Phase::Showing || sess.phase == Phase::Replaying)) {
            bool showed = sess.phase == Phase::Showing;
            endSession(fmt::format("{} cleared the level at {:.2f}x.", showed ? "The solution" : "The stored solution",
                                   g.watchSpeed), Phase::Passed);
            g.status = "Solved.";
            if (g.record && g.autoSave && !rt.inputs.empty()) saveReplay(this);
        }
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

        if (rt.pendingShow) {                      // the solution cleared: replay it at 1x
            rt.pendingShow = false;
            sess.phase = Phase::Showing;
            setTimeScale(g.watchSpeed);
            this->resetLevel();
        }

        // the fast dark loop: nothing to draw while the search is running
        bool dark = sess.active && g.dark && sess.phase == Phase::Solving;
        if (m_objectLayer) m_objectLayer->setVisible(!dark);

        if (rt.label) {
            rt.label->setVisible(sess.active);
            if (sess.active) {
                float lvl = lmap.xEnd > 0 ? (float)rt.trace.size() * 0.f : 0.f;
                (void)lvl;
                int tickNow = (int)rt.trace.size();
                float endTick = rt.pred.empty() ? 1.f : (float)rt.pred.size();
                std::string txt = fmt::format("Pathfinder  iter {}  {}  {:.0f}s\n"
                                              "level  [{}]  {:.1f}%   best {:.1f}%\n"
                                              "plan   [{}]  tick {} / {}   fixups {}\n{}",
                    sess.attempt + 1, phaseName(sess.phase), elapsedSeconds(),
                    barStr(g.percent / 100.f), g.percent, sess.best,
                    barStr(tickNow / endTick), tickNow, (int)endTick, (int)rep.fix.size(), rt.hudNote);
                rt.label->setString(txt.c_str());
            }
        }

        auto p = m_player1;
        if (!p || p->m_isDead) return;
        g.modeName = modeName(modeOf(p));

        if (g.overlay && rt.draw && !dark) {
            float px = p->getPositionX();
            auto it = std::lower_bound(lmap.hz.begin(), lmap.hz.end(), px - 60.f,
                [](CCRect const& r, float v) { return r.getMinX() < v; });
            for (int n = 0; it != lmap.hz.end() && it->getMinX() < px + 600.f && n < 60; ++it, ++n) {
                rt.draw->drawDot({it->getMidX(), it->getMidY()}, 7.f, {1.f, 0.2f, 0.2f, 0.7f});
            }
            if (sess.active && !rt.pred.empty()) {
                int from = std::max(0, (int)rt.trace.size());
                for (int i = from; i < (int)rt.pred.size() && i < from + 720; i += 12) {
                    rt.draw->drawDot({(float)(rt.pred[i].x + rt.pred[i].w * 0.5), (float)(rt.pred[i].y + rt.pred[i].h * 0.5)},
                                     2.5f, {0.2f, 1.f, 0.4f, 0.8f});
                }
            }
        }
    }
};
