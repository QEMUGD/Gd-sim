# Pathfinder
By [qemugd](https://github.com/qemugd) · [Source on GitHub](https://github.com/qemugd/gd-sim)

Plays a level's **cube sections** with a physics look-ahead, records every input
on its exact 240 TPS physics frame, and exports a **.gdr2** replay you can load
in **Eclipse Menu** (or any GDR2 bot).

Tap the logo (main menu bottom bar / bottom-right of the pause menu) to open the panel.

## Use
1. Open a level, enable *Drive cube* and *Record*.
2. Finish the level. With *Auto-save* on, a `.gdr2` is written when you complete it
   (or press **Save .gdr2 now** at any time).
3. If Eclipse is installed the file goes straight into Eclipse's `replays` folder;
   otherwise it goes in this mod's save folder.

## Credits
- **[qemugd](https://github.com/qemugd)** — author.
- **[camila314](https://github.com/camila314)** — GD physics decompilations
  ([camila314/gdp](https://github.com/camila314/gdp)). The predictor's constants and
  formulas (per-speed jump/gravity values, mini scaling, fall clamp, 240 Hz stepping)
  come from her `updateTimeMod`, `updateJump` and `GJBaseGameLayer::update`.
  Used with her permission.
- **[maxnut](https://github.com/maxnut/GDReplayFormat)** — GDReplayFormat (GDR2).
- [matcool/gd-imgui-cocos](https://github.com/matcool/gd-imgui-cocos) — ImGui for Cocos2d.
- Built on [Geode](https://geode-sdk.org).

## Limits
Cube mode only; hazards are modelled, blocks/orbs/pads/portals are not, so it won't
beat most real levels on its own. Runs that used practice-mode respawns are flagged
because they may not replay cleanly.
