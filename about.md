# Pathfinder
By [qemugd](https://github.com/qemugd) · [Source on GitHub](https://github.com/qemugd/gd-sim)

Open a level, tap the **cube button** on its page, pick your settings, press **OK**.

## What it does
1. **Maps** the level: hazards, solid blocks and portals (gamemode, gravity, size, speed).
2. **Plans** a whole-level macro in a physics simulation, using taps *and holds*.
3. **Verifies** the macro by replaying it separately in the simulation. If planning or
   verification fails it retries with other safety margins.
4. **Plays** the verified macro in the real game. If the real run drifts from the
   prediction, it re-plans from the real state.
5. If you **die**, it logs the attempt (percent, position, killer object), adds caution
   around that spot, re-maps and plans again.
6. When the level is **passed**, the inputs are saved as a **.gdr2** replay for
   **Eclipse Menu** (or any GDR2 bot).

## Settings (in the dialog, and the Settings tab)
- **Game speed** 1x–10x (default 10x while pathfinding)
- **TPS** 240–5000 (240 = vanilla). Above 240 is experimental and heavy
- **CPS lock** (on by default, 16 clicks per second). Turn it off *before* starting
- **Allow holding** (on by default). Off = taps only
- Max attempts, planner width, safety margin
- Record / auto-save / overlay

## Panel
Tap the Pathfinder logo (main menu bottom bar, or bottom-right of the pause menu):
Status, Settings, Map (what was mapped + caution zones) and Log tabs. The log is also
written to `pathfinder_log.txt` in the mod's save folder.

## Credits
- **[camila314](https://github.com/camila314)** — GD physics decompilations
  ([camila314/gdp](https://github.com/camila314/gdp)). Per-speed jump/gravity values,
  per-vehicle rules, clamps and step logic come from her `updateTimeMod`, `updateJump`,
  `GJBaseGameLayer::update` and `PlayLayer::destroyPlayer`. Used with her permission.
- **[maxnut](https://github.com/maxnut/GDReplayFormat)** — GDReplayFormat (GDR2).
- [matcool/gd-imgui-cocos](https://github.com/matcool/gd-imgui-cocos) — ImGui for Cocos2d.
- Built on [Geode](https://geode-sdk.org).

## Limits
All gamemodes are modelled (cube is the most reliable). Orbs, pads and dual/mirror
portals are **not** modelled, so levels that need them will fail, retry and log.
Classic levels only (no platformer).
