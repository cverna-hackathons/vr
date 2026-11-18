# What
Directory should contain source for an application. The application should allow user to utilize an Oculus DK2 headset with it's peripherals.

This will run under Linux Mint distribution. Current target machine is Lenovo Carbon X1 Gen 12.

## Progressive objectives
1. Connect to the headset hardware and show one color (dark blue) across the whole screen area.
2. Connect to the peripheral position tracking sensor. Change color on screen area based on the relative rotation of headset: If wearer turns left, gradually change to dark red, for right, gradually change to dark green. Change back to dark blue for center.
3. Render flat white text within the screen area.
4. Render a 3D scene, with a look at a cube from isometric perspective.
5. Reuse scene #4 and allow the position tracking sensor to provide data for view adjustment so that the wearer can observe the point of view rolling, closing in and other movement.
6. Connect to the peripheral position tracking camera to also obtain more accurate information about the relative distance updates, rotation and position. Use that for scene from #5.
7. Add an image file (under media/) on the face of displayed cube, and render that into scene from #6.
8. Add a video file (under media/) on another faceo for the displayed cube and render it.
9. Add a theater scene that will place a proper big screen in front of the view and allow the spectator to watch a full movie. Also play sound with this one. Keep the execution separate from other objectives, and so that it allows a video file to be specified, like so: `./build/theater --video /path/other.mp4`
10. Add a virtual display scene, which when run, allows to mirror all content that displays on the built-in display as if it was a monitor in front of the user. User should be able to operate all input as usual, and only Ctrl+X short cut should exit that mode and terminate the program. Should be possible to run under `./build/display`.
11. Load a text with a symbol description of a map and objects on the map. Allow user to load a custom txt file with a map. Define a symbol glossary for each symbol representing specific thing on the map. With `./build/explore`, allow user to explore the map represented in 3D. If there are already systems for this textual map representation, you could use those.
12. For each of the scenes / modes, allow to pass --no-vr to run the scene without the VR headset. During that mode, allow keyboard for movement tracking input.

# How
It isn't important to expect anything fancy, as the spec will be below average and mostly integrated graphics, CPU and RAM will do all the heavy lifting. There isn't a dedicated graphics card available.

# Why
Attempt using old VR headset from Linux for simple scene rendering.


# Build & run
```
sudo apt install build-essential cmake pkg-config libsdl2-dev libgl-dev libhidapi-dev \
    libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libswresample-dev
cmake -S . -B build && cmake --build build -j
./build/vr --list        # show displays; the DK2 appears as a 1080x1920 or 1920x1080 output
./build/vr               # auto-picks the DK2-sized display; Esc quits
./build/vr --display 1   # or choose explicitly
```
Use an X11 session. If the DK2 is not detected by OpenHMD, check USB permissions (udev rule for 2833:0021).

## DK2 display bring-up (after each boot/replug)
The kernel hides the DK2 as a "non-desktop" VR output, and the laptop's HDMI PHY rejects the DK2's native 75 Hz clock.
```
sudo tools/dk2-edid-override.sh   # patch EDID vendor/product so X sees the output; then replug HDMI
tools/dk2-setup.sh                # (vr, theater and display run this automatically at startup; pass --no-setup to skip) define and enable 1080x1920 @ 60 Hz (148.5 MHz) right of the laptop screen; does nothing if already set up (--force to redo)
./build/vr
```
Status: objectives 1-5 verified on the headset; 6 implemented (needs verification). Runs at 60 Hz, not the native 75 Hz.

## Tracking architecture (objective 6)
OpenHMD is no longer used. `src/dk2_hid.*` talks to the headset directly (IMU, keepalive, LED blink control), because the LED
blink mode needed to identify LEDs can only be set by the process that owns the HID connection. Only one program can own the
headset at a time, so close `./build/vr` before running any `tools/` program that opens it.
- `src/imu_fusion.hpp`: gyro + accelerometer orientation (Mahony filter).
- `src/ptc_camera.*`: PTC capture, sensor sync-exposure setup, lens calibration read from the camera EEPROM.
- `src/ptc_tracker.*`: finds LED dots, tracks them, identifies each by its 10-frame blink pattern (`data/dk2_leds.txt`).
- `src/ptc_pose.*`: undistort + perspective-n-point solver (Levenberg-Marquardt).
- `src/head_tracker.*`: merges IMU orientation with camera position and camera yaw-drift correction.
Tools: `ledlayout` (read LED layout from headset), `ptc_ids`, `ptc_pose_test`, `track_debug`, `camprobe`, `dk2_imu`.
`./build/vr --no-camera` runs with orientation only.

## Modes and keys
`2` colour shift, `3` colour shift + text, `4` stereo cube (fixed isometric camera), `5` stereo cube with head-tracked camera (default).
Mode 5: W/S forward/back, A/D strafe, Q/E down/up (Shift = slow), R reset keyboard offset, Space/C re-centre heading and position, Esc quit.
The window must have keyboard focus; if keys do nothing, click it or move the mouse pointer over the headset display.

## Theater (objective 9)
Separate executable, independent of the cube scenes: a large virtual screen (6 m wide, 4.5 m in front of you) in a dark room with a
faint floor grid. Head rotation and PTC position tracking work as in mode 5. Sound plays through the default audio device.
```
./build/theater                          # plays media/video.mp4
./build/theater --video /path/other.mp4  # any file FFmpeg can read
```
Keys: Space/P pause, Left/Right seek -/+10 s (Shift: 60 s), Up/Down volume, M mute, C re-centre, F1/F2/F3 flip tracked position
axes, Esc quit. A progress/volume bar appears under the screen for 3 s after a key press (and while paused).
Picture is paced by the audio clock (`src/movie.*`: demux, audio and video threads, software decoding, scaled to <= 1280 px wide).

## Virtual display (objective 10)
`./build/display` mirrors the laptop's built-in screen (RandR output `eDP*`) onto a large screen curved horizontally around you
(a cylinder section, default 110 degrees wide at 1.5 m), with head rotation and PTC position tracking, in the same dark room as the theater.
Input is confined to the mirrored screen, so it behaves like a real monitor in front of you:
- the mouse pointer is drawn into the picture (magnified, it is tiny in the headset) and X pointer barriers keep it from leaving the
  built-in screen, so it is never somewhere you cannot see (this also blocks the external monitor while running);
- keyboard focus is kept on a window on that screen (checked twice a second; if the active window is elsewhere, the top-most window on the
  mirrored screen is activated), so typing always lands in something you can see;
- the headset window is override-redirect and never takes focus.
Global hotkeys (grabbed while it runs, so Ctrl+X no longer cuts text in other programs until you exit; it is also detected from the
physical key state in case another program holds a keyboard grab): `Ctrl+X` exit and terminate, `Ctrl+F9` re-centre (also automatic 4 s
after start), `Ctrl+F10` / `Ctrl+F11` screen wider / narrower.
```
./build/display                          # mirror eDP-1 (default)
./build/display --source DP-3            # mirror another output
./build/display --arc 140 --distance 1.3 --cursor 3 --fps 60
```
Capture is XShm at 30 fps (`src/xcapture.*`).

**Gaze lens.** The DK2 resolves only ~10 pixels per degree, so a 12-pixel font is a blur when the whole screen is in view. The lens magnifies the part of
the screen you are looking at (an elliptical region, about 1 m wide, outlined in yellow, magnified 2.2x) using the capture's full resolution, so small
text becomes readable. It follows where your head points on the curved screen (smoothed, so it doesn't jitter) and disappears when you look away from the
screen. It starts off: `Ctrl+F12` toggles it, `--lens` starts with it on; `--lens-zoom Z` and `--lens-size M` change magnification and width. Raising the laptop's
display scale in Cinnamon (150-200 %) helps as well. (`display --shot FILE.ppm --shot-gaze U V` renders one frame of a synthetic test desktop for checking.)

## Explore (objective 11)
`./build/explore` loads a text map and lets you walk through it in 3D on the DK2 (1 cell = 1 m, walls 2.5 m, eye height 1.7 m).
```
./build/explore                                     # data/maps/demo.txt (small: rooms, garden, pond)
./build/explore --map data/maps/valley.txt          # large: 100 x 44 m, hills, river, lake, village, ruined castle, forest
./build/explore --map my.txt                        # your own map
./build/explore --map my.txt --glossary mine.txt    # plus your own symbol glossary
./build/explore --check --map my.txt                # validate and report (no headset needed); also runs collision and crosshair self-tests
./build/explore --reach 8                           # crosshair / interaction distance in metres (default 5; 0 = no crosshair, no interaction)
./build/explore --print-glossary                    # print the built-in glossary, to copy and edit
```
Keys: M mute, `-` / `=` volume, W/S walk (in the direction you look), A/D strafe, Shift run, Left/Right arrows turn the body, **F** fly mode (Space up, Left Ctrl down,
W/S fly along your gaze, no collisions), C re-centre, F5 reload the map file (edit and look again without restarting), E or left mouse button interact with what the crosshair points at, F1/F2/F3 flip the tracked position axes (saved for all programs), Esc quit. Leaning with the PTC moves your head, but not through walls. The console prints each cell you enter.

**Map files** use the roguelike ASCII convention (NetHack/Rogue style): one character per cell, top of the file is north. A map file can
be just the grid, or have sections: `@glossary` (symbol definitions) and `@map` (the grid; inside it `#` is a wall, not a comment).
Outside the map, `;` starts a comment line. Symbols can be redefined or added with `--glossary FILE` and/or an `@glossary` section
(the map's own section wins, then `--glossary`, then the built-ins). Undefined symbols are drawn as a magenta post and reported.

Glossary line syntax: `<symbol> <kind> [key=value ...] | label`, and `space <kind> ...` for the space character. Kinds: `wall` `floor`
`water` `tree` `rock` `pillar` `door` `item` `start` `empty` `model`. Keys: `color=#rrggbb`, `color2=#rrggbb` (tree trunk / door posts),
`floor=#rrggbb|no` (ground under the object), `height=metres`, `walk=yes|no`. Scene options: `option sky=#rrggbb fog=0.03 ceiling=yes ceiling_height=2.5`.

| Symbol | Kind | Meaning | Walkable |
|---|---|---|---|
| `#` | wall | stone wall, 2.5 m | no |
| `W` | wall | wooden wall | no |
| `H` | wall | hedge, 1.4 m | no |
| `.` | floor | stone floor | yes |
| `,` | floor | grass | yes |
| `:` | floor | sand | yes |
| `~` | water | water | no |
| `T` | tree | tree, 3.2 m | no |
| `o` | rock | boulder, 0.9 m | no |
| `I` | pillar | pillar, 2.5 m | no |
| `+` | door | doorway (posts and lintel orient themselves to the walls beside it) | yes |
| `@` | start | start position (default: first walkable cell) | yes |
| `*` | item | treasure (floating gem) | yes |
| space | empty | void, no floor | no |

**Custom models (glTF).** `<symbol> model file=data/objects/thing.glb [scale=1] [rotate=0] [yoffset=0] [walk=no] | label` draws a glTF 2.0
file (`.glb` or `.gltf`, e.g. exported from Blender) at the centre of the cell, standing on the terrain. `file` is looked up as given, then
relative to the map file and its two parent folders, so `data/objects/...` works from `data/maps/`. Units are metres, Y up (Blender's
default glTF export), the model's origin is placed at the cell centre on the ground (so put the origin at the foot), and it faces +Z
(south) at `rotate=0`; `rotate` is in degrees about the vertical axis. The model blocks its whole cell unless `walk=yes`. Supported: the
node hierarchy, triangle meshes, the material's base colour and base-colour texture (PNG/JPEG, mip-mapped), and alpha MASK/BLEND as a
cut-out (leaves, fences). Not supported: animation, skins, normal/roughness/emissive maps (lighting is the same simple sun + ambient as
the rest of the map). `explore --check` prints each model's triangle count, texture count and size so you can pick `scale`. A missing
or broken file stops the map from loading with a clear message (F5 keeps the old map). Keep the source `.blend` next to its export in `data/objects/` (`crate.blend` + `crate.glb`). Add viewport-only scale helpers (the 1 m
map cell and a grid of cells, a 1.75 m person, a ruler to 3.5 m, and an arrow showing the front, which is explore's south = Blender's -Y)
and a glTF collection exporter to any model `.blend` with `blender --background data/objects/mything.blend --python tools/blender_helpers.py`; `tools/new_model.sh NAME` clones the crate into `data/objects/NAME.blend` with its exporter, helpers and `NAME.glb` already set up;
the helpers sit in a separate render-disabled collection, so they never reach the `.glb`, and File > Export All Collections in Blender
re-exports it. `tools/blender_make_sample.py` generated the sample crate and is a template for your own models; it refuses to overwrite
an existing `crate.blend`.

**Crosshair and interaction.** A small semi-transparent dot inverts the colours behind it, on your line of sight. It sits at the depth of whatever
it points at (a wall, the ground, or `--reach` metres away in open air), so both eyes converge on it in the headset. It is larger when the
object it points at is interactable, and flashes when you press **E** (or the left mouse button) to interact; the console prints
`Interact: <label> (<kind> '<symbol>') at cell (x,z), D m away`, and `Nothing to interact with within D m` if nothing is in reach.
Walls, solid objects and the ground block the line of sight, so only what is directly visible within `--reach` (default 5 m) can be used.
Which glossary entries can be used: `door`, `item`, `tree`, `rock`, `pillar` and `model` by default; override with `interact=yes|no`
(e.g. `Q model file=... interact=no`). Doors count when you point at the lintel (the opening is see-through). Objects are tested as a
square footprint with a height range, so the test is generous at the edges. Inversion cannot show against a mid-grey (50 %) background.
The interaction itself is only that message and the flash for now; `src/explore.cpp` (search for "Interact:") is the place to hook behaviour in.

**Terrain and slopes.** Add an `@height` section: a second grid, same size and layout as the `@map`, giving the elevation of every cell
as a digit `0`-`9` (then `a`-`z` = 10-35; `.` or space = 0), in steps of `option height_step=metres` (default 0.5 m). The ground is a smooth
surface through those heights, so a slope appears wherever neighbouring cells differ. Trees, rocks, pillars, doors and items stand on it, walls
follow it, shores slope down to the water surface, and your eyes stay 1.7 m above the ground as you walk up and down. Walking uphill steeper than
about 50 degrees is blocked (a cliff), downhill is free. Keep neighbouring cells within one or two levels for walkable slopes. Example:
```
@map          @height
#######       .......
#.....#       ..123..      <- a hill in the middle of the room
#.....#       .12321.
#######       .......
```
`./build/explore --check` prints the elevation range. A map without `@height` is flat, as before.

**Sound.** The world is audible in 3D (use headphones). Sounds are synthesised at start-up, so no sound files are needed. Water cells give a low, slow
flowing undertone (no bubbling), trees rustle softly (about one in six has a bird), gems hum a soft chord so you can find them by ear. A very quiet low air bed runs underneath. Each source is placed in space: it reaches the nearer
ear first (up to 0.66 ms earlier) and louder, the far ear is duller (head shadow), sources behind you are dulled, level falls with distance (nothing
beyond 18 m), and a wall between you and a source makes it quieter and muffled. Turn your head and the sources move accordingly. Walking makes
footsteps that depend on the ground: stone is a soft clap with a hint of gravel being ground, grass a soft muffled brush, sand a gentle scuff, wood a muted knock.
Every kind has six different variants, with random level, pitch and stride spacing, so steps never repeat exactly. Grass has no thud at all, just a slow, soft brush.
Step spacing follows your body: `--height M` (default 1.9) sets your height, and with it the eye height (height - 12 cm) and step length (0.5 x height walking,
0.82 x height running), giving 126 steps/min walking and 173 running at the default speeds (2.0 and 4.5 m/s). Glossary keys: `sound=none|water|leaves|bird|hum|fire` (any symbol can be a
sound source, e.g. `L fire` for a torch: `L pillar color=#ff9040 height=1.2 sound=fire | Torch`) and `step=none|stone|grass|sand|wood`.
The nearest 12 sources are mixed (`src/spatial_audio.*`).
`./build/sound_demo DIR` renders the sounds to WAV files (footsteps for each ground, river, forest, gems, ambience) so you can audition and compare them
without the headset, and prints their levels; the synthesis recipes are in `SpatialAudio::synthesise()`, all gains and filters in one place.

The demo map defines `R` (ruby), `F` (fountain basin) and `B` (low brick wall) in its own `@glossary`. Code: `src/mapdata.*` (parser,
collision, terrain, mesh), `src/gltf_model.*` (glTF loader, tinygltf in `third_party/tinygltf`) and `src/explore.cpp`. The `valley.txt` map was generated by a script, but it is plain text like any other: edit it and press F5.

## No headset (`--no-vr`, objective 12)
Every scene accepts `--no-vr`: `./build/vr --no-vr`, `./build/theater --no-vr`, `./build/display --no-vr`, `./build/explore --no-vr`. The headset, its tracking and the
DK2 display setup are not touched. The scene is drawn as one ordinary perspective view (75 degrees vertical, no lens distortion) in a resizable window, and the
keyboard stands in for head tracking:

| Keys | Effect |
|---|---|
| Ctrl + Left / Right | turn the head left / right |
| Ctrl + Up / Down | look up / down |
| Ctrl + Shift + Left / Right | lean left / right |
| Ctrl + Shift + Up / Down | lean forward / back |
| Ctrl + PageUp / PageDown | lean up / down |
| Ctrl + Backspace | back to looking straight ahead, no lean |
| add Alt | slower, finer movement |

Everything else keeps its normal keys (W/A/S/D walking in `explore`, arrows for seeking in `theater`, and so on); the plain arrows only act when Ctrl is not held.
In `display --no-vr` the virtual monitor opens in a window covering another display (`--display N` chooses one; with a single display it opens as a smaller window
on the mirrored screen and shows itself). Because keyboard input stays confined to the mirrored screen there, the head keys are grabbed globally, so applications
on the mirrored screen do not see Ctrl+arrows, Ctrl+PageUp/PageDown or Ctrl+Backspace while it runs. The gaze lens (`Ctrl+F12`) follows where the keyboard head points.

## Latency and comfort
Every program predicts the head pose forward by 20 ms (`HeadTracker::set_prediction`): the gyro's angular velocity extrapolates the orientation to
the moment the light reaches your eye, which cancels most of the lag during head turns (at 120 deg/s, 20 ms is 2.4 degrees). Tune it by feel with
`--predict MS` (0 disables) or, in `vr`, `theater` and `explore`, with the `[` and `]` keys (2 ms per press, printed on the console): too
little feels like the world lagging behind a head turn, too much feels like it overshooting and wobbling. The prediction is capped so a gyro glitch
cannot fling the view. When the PTC loses the headset, the tracked position eases back to the centre over about a second instead of freezing at a
stale value, and eases in again when the camera finds the headset.

## Audio device changes
`explore` and `theater` survive another application taking over or changing the sound output (a notification that switches a Bluetooth headset's
profile, the headset disconnecting, the default device changing). SDL does not stop its audio callback when a device disappears; it keeps calling it and
discards the sound, so the programs listen for SDL's "device removed" event (and for the callback stopping altogether) and reopen the default output within
a fraction of a second, printing `Audio: output restored.` Playback state is kept (the movie carries on at the same position; the world keeps its sounds).
If no output exists at all, they keep retrying every 2 seconds. The sound then comes out of whichever device is the default at that moment.

## Camera mirror (why poses are now correct)
The PTC delivers a left-right **mirrored** picture relative to the LED model. This used to be patched after the pose solve with two "mirror" constants,
which cannot work when only the flat front plate is visible (a flat pattern fits a flipped pose; the solver found no valid pose at all for a headset held
face-on, with perfect 18-LED identification). `PtcPoseSolver` now mirrors the observations back before solving, giving proper rigid poses (0.14 px fit, 96 %
of frames valid in that case, versus 0 %), and the tracker's two mirror constants are the identity. Tracked positions then match reality without any sign
flip, so the default position signs are now +1 +1 +1 (they were -1 -1 -1 to compensate the flipped poses). `DK2_LEGACY_MIRROR=1` restores the old behaviour
(old solver input, mirror constants, signs -1) for comparison. `lean_calibrate` checks this: its rotation phase compares the camera's rotation with the IMU's
while you turn and also tests every candidate mirror mapping against your recording.

## Position directions (leaning left/right, forward/back, up/down)
The camera's position axes must move the same way as your head. This is decided once, and stored in `~/.config/dk2vr/position_signs`, which every program
reads (`vr`, `theater`, `display`, `explore`), so they can never disagree. If leaning feels reversed:
- **Measure it (recommended):** close the other programs and run `./build/lean_calibrate` (it starts with a rotation check, then the three lean phases). Hold the headset in front of your face as if wearing it, facing
  the camera, and follow the three 8-second phases (toward/away from the camera, left/right, up/down, about once a second, 10-20 cm strokes). It uses the
  headset's accelerometer as ground truth: for a back-and-forth motion the acceleration is opposite to the position, so a correct axis correlates
  negatively with it and a reversed one positively (the camera's delay of up to 250 ms is found automatically). Only axes with enough movement and a clear
  result are saved; the others keep their sign and the tool tells you to repeat. `--dry-run` shows the result without saving.
- **Or flip by feel:** F1/F2/F3 (X left/right, Y up/down, Z forward/back) in `vr`, `theater` and `explore`, `Ctrl+F6/F7/F8` in `display`. The new signs are saved too.
`src/lean_calibration.hpp` holds the analysis (tested against synthetic motion with camera lag, noise and tilt error).
