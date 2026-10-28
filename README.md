# Pathfinder (Geode mod)
By [qemugd](https://github.com/qemugd) - source: https://github.com/qemugd/gd-sim

Hazard-aware cube path assist + ImGui panel for Windows and Android.

## Build
Easiest: push this folder to a GitHub repo -> Actions -> "Build Mod" -> download
the `.geode` file. Put it in your Geode `mods` folder (or install via Geode's
"Install from file" on Android).

Local: install Geode CLI, `geode sdk install`, then `geode build`.

## Use
Tap/click the Pathfinder logo (main menu bottom bar, or bottom-right of the pause menu).
Finish a level with the assist/recording on; a `.gdr2` is written to Eclipse's replays folder
(or this mod's save folder). Load it in Eclipse's replay tab.

## Credits
- **camila314** - GD physics decompilations ([camila314/gdp](https://github.com/camila314/gdp)); the
  predictor's constants/formulas come from `updateTimeMod`, `updateJump` and `GJBaseGameLayer::update`.
  Used with her permission.
- **maxnut** - [GDReplayFormat](https://github.com/maxnut/GDReplayFormat) (GDR2).
- [matcool/gd-imgui-cocos](https://github.com/matcool/gd-imgui-cocos) - ImGui for Cocos2d.
- [Geode SDK](https://geode-sdk.org).

## Limits
Cube mode only; hazards are modelled, blocks/orbs/pads/portals are not.
