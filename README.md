# CameraSmooth — SnowRunner free camera + rotation smoothing

A native ASI plugin for SnowRunner that:

- enables the developer **free camera** with a hotkey (same mechanism as
  [SnowFlyer2](https://github.com/NakedDevA/SnowFlyer2) — no external trainer needed);
- **smooths camera rotation** — the jerky, tearing free-camera movement is replaced
  by smooth exponential interpolation (quaternion slerp) applied to the camera's
  real transform state every frame. Not motion blur, not input sensitivity —
  the camera matrices themselves are smoothed after the game writes them and
  before the renderer reads them.

## Requirements

- SnowRunner (Steam/Epic, x64) — tested on the Season-19-era build.
- An ASI loader. If you already use mods like `SMT.asi` / `Spotter.asi`,
  you have one (`dinput8.dll` in the game directory).

## Install

1. Download `CameraSmooth.asi` from Releases.
2. Copy it into the game directory next to the ASI loader:
   `...\SnowRunner\Sources\Bin\` — the folder that contains `dinput8.dll`.
3. Launch the game. The mod loads automatically.

## Controls

| Key | Action |
|-----|--------|
| **F9** | Toggle free camera on/off |
| **F8** | Toggle camera smoothing on/off (on by default) |

Camera rotation is smoothed with an exponential slerp (tau = 120 ms).
Camera position is never smoothed — only orientation.

## How it works

The plugin detours two points in the game's per-frame camera update:

- the epilogue of the camera-matrix writer (normal / photo-mode path);
- the merge point after the dev-freecam update block in the frame function.

At each point it reads the freshly computed camera basis (world matrix),
converts it to a quaternion, slerps it toward the current target orientation,
and writes the smoothed view + world matrices back — on the game's own thread,
so there is no race with the renderer. Projection/TAA-jitter blocks are never
touched.

## Build

```bash
x86_64-w64-mingw32-g++ -O2 -shared -static -o CameraSmooth.asi src/CameraSmooth.cpp
```

Source: `src/CameraSmooth.cpp`. No external dependencies (Win32 only).

## Credits

- Free-camera enable logic based on [SnowFlyer2](https://github.com/NakedDevA/SnowFlyer2)
  by NakedDevA (dev-check patch + fly-mode flag), reimplemented natively.
