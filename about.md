# Pathfinder
By [qemugd](https://github.com/qemugd) · [Source on GitHub](https://github.com/qemugd/gd-sim)

Open a level, tap the **cube button** on its page, pick your settings, press **OK**.
Pathfinder maps the level, plays it with a physics look-ahead, logs every failed
attempt, re-maps with extra caution around the failure and tries again until it
passes. Every input is recorded on its exact physics frame and saved as a **.gdr2**
replay for **Eclipse Menu** (or any GDR2 bot).

## Settings (in the dialog, and the Settings tab)
- **Game speed** 1x–10x (default 10x while pathfinding)
- **TPS** 240–5000 (240 = vanilla). Above 240 is experimental and heavy
- **CPS lock** (on by default, 16 clicks per second). Turn it off *before* starting
- Max attempts, look-ahead, safety margin, click cooldown
- Record / auto-save / overlay

## Panel
Tap the Pathfinder logo (main menu bottom bar, or bottom-right of the pause menu):
Status, Settings, Map (hazards/blocks mapped + caution zones) and Log tabs.
The log is also written to `pathfinder_log.txt` in the mod's save folder.

## Credits
- **[camila314](https://github.com/camila314)** — GD physics decompilations
  ([camila314/gdp](https://github.com/camila314/gdp)). Per-speed jump/gravity values,
  per-vehicle rules, clamps and step logic come from her `updateTimeMod`, `updateJump`,
  `GJBaseGameLayer::update` and `PlayLayer::destroyPlayer`. Used with her permission.
- **[maxnut](https://github.com/maxnut/GDReplayFormat)** — GDReplayFormat (GDR2).
- [matcool/gd-imgui-cocos](https://github.com/matcool/gd-imgui-cocos) — ImGui for Cocos2d.
- Built on [Geode](https://geode-sdk.org).

## Limits
All gamemodes are modelled (cube is the most reliable). Hazards and solid blocks are
mapped; orbs, pads and portals are not, so levels that need them will fail and log.
Classic levels only (no platformer).
