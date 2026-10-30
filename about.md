# Pathfinder
By [qemugd](https://github.com/qemugd) · [Source on GitHub](https://github.com/qemugd/gd-sim)

Works like [gdsolver](https://github.com/gdsolver/gdsolver): a planner *proposes* an input
plan from a model of the game, and the game itself *verifies* it by playing it.

Open a level, tap the **cube button** on its page, pick your settings, press **OK**.

## The loop
1. **Map** the level: hazards, solid blocks and portals (gamemode, gravity, size, speed).
2. **Plan** a whole-level macro with a reachability search over 240 Hz ticks (taps and holds).
3. **Replay** the plan in the game, fast and dark, dumping the game's real state every tick.
4. When the game **dies**: find the first tick where the game and the model disagree and
   record a **fixup**; **re-anchor** on the game's real state a few ticks before the failure
   (a ladder of depths); plan the tail again and pin everything before it. If the ladder
   runs out, **search that section in the game** by perturbing the inputs.
5. When a plan **clears**, the level restarts and the solution is shown at 1x. It is saved
   as `solution_<id>.txt` (`input=<tick>,<0|1>` lines) and as a **.gdr2** replay for
   **Eclipse Menu** (or any GDR2 bot).

## Settings (in the dialog, and the Settings tab)
- **Mode**: solve the level, or replay the stored solution
- **Solving speed** 1x–10x (default 10x) and **watching speed** 0.25x–4x
- **Fast dark loop**: hide the level while solving
- **TPS** 240–5000 (240 = vanilla). Above 240 is experimental and heavy
- **CPS lock** (on by default, 16 clicks per second). Turn it off *before* starting
- **Allow holding** (on by default). Off = taps only
- Max attempts, planner width, first-plan margin, record / auto-save / overlay

## Panel
Tap the Pathfinder logo (main menu bottom bar, or bottom-right of the pause menu):
Status, Settings, Map (what was mapped, fixups, failure spots) and Log tabs. The log is
also written to `pathfinder_log.txt` in the mod's save folder.

## Safety
While the bot drives, the level is played in the game's test mode (its own "do not
record" path), so your progress, stats and the level's record are not touched.
Your own attempts, with the bot idle, record as usual. This is a TAS / research tool.

## Credits
- **[gdsolver](https://github.com/gdsolver/gdsolver)** (MIT) — the approach: reachability
  planner proposes, the game verifies, repair by re-anchoring on the game's own state,
  fixups, an anchor-depth ladder, a section solver, a fast dark loop and a 1x showing.
  This mod is a much smaller, independent implementation of those ideas.
- **[camila314](https://github.com/camila314)** — GD physics decompilations
  ([camila314/gdp](https://github.com/camila314/gdp)). Per-speed jump/gravity values,
  per-vehicle rules, clamps and step logic come from her `updateTimeMod`, `updateJump`,
  `GJBaseGameLayer::update` and `PlayLayer::destroyPlayer`. Used with her permission.
- **[maxnut](https://github.com/maxnut/GDReplayFormat)** — GDReplayFormat (GDR2).
- [matcool/gd-imgui-cocos](https://github.com/matcool/gd-imgui-cocos) — ImGui for Cocos2d.
- Built on [Geode](https://geode-sdk.org).

## Limits
All gamemodes are modelled (cube is the most reliable). Orbs, pads and dual/mirror portals
and moving geometry are **not** modelled, so levels using them rely on the repair loop and
section search and may not solve. Classic levels only (no platformer).
