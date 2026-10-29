# spectaGATR

Live robot viewer. The Brain program sends the pose it already has, the
robot's size and the path it plans to drive; a laptop program passes them to a
browser that draws the field, the robot, its trail and the path. spectaGATR does
no localization and never moves the robot.

```
Brain program --stdout--> V5 controller (radio) --USB--> laptop: receiver --> browser
  brain/                  or Brain USB port                   receiver/        web/
```

## Setup

Three steps: build the receiver on the laptop once, add the library to the PROS
project, call it from the program.

### 1. Build the receiver (laptop, once)

Needs CMake 3.16 or newer and a C++17 compiler. On Windows that is MSYS2 ucrt64
GCC, with `C:\msys64\ucrt64\bin` first on PATH while building. From the
GATR2-Override folder:

```sh
cmake -S lib/spectaGATR -B build-spectagatr -G "MinGW Makefiles"
cmake --build build-spectagatr --parallel
build-spectagatr/receiver/spectagatr --source sim
```

The last line runs a simulated robot and opens the viewer at
<http://127.0.0.1:8790/>. If a robot drives a figure eight on the field, the
receiver works. On Linux and macOS leave out `-G "MinGW Makefiles"`.

### 2. Add it to the PROS project

In the season project's `Makefile`, above the "Nothing below this line" block:

```make
GATR2_LIB_ROOT ?= ../lib
GATR2_LIBS := spectagatr
```

and at the bottom, `lib.mk` on both sides of `common.mk`:

```make
include $(GATR2_LIB_ROOT)/lib.mk
-include ./common.mk
include $(GATR2_LIB_ROOT)/lib.mk
```

`2026-27/Makefile` already has both. `GATR2_LIB_ROOT` is `lib/` relative to the
project folder, with no drive letter.

### 3. Use it in the program

```cpp
#include "main.h"
#include "spectagatr/viewer.h"

spectagatr::ViewerConfig vizConfig() {
    spectagatr::ViewerConfig c;
    c.robot = spectagatr::RobotBox::centered(0.38, 0.33, 0.25); // length, width, height in m
    return c;
}

spectagatr::Viewer viz(vizConfig());

void initialize() {
    viz.start();
}

void autonomous() {
    const std::vector<spectagatr::Point> path = {{0.6, 0.6}, {1.8, 0.6}, {1.8, 1.8}};
    viz.setPath(path); // field meters, drawn point to point
    // drive it
    viz.clearPath();
}

void opcontrol() {
    while (true) {
        const double x = 1.0, y = 0.5, heading = 0.0; // from your localization
        viz.updatePose(x, y, heading);                // meters, radians CCW from field +x
        pros::delay(10);
    }
}
```

That is everything the program needs:

- `viz` is a global, created once.
- `viz.start()` goes in `initialize()`, never in a global constructor.
- `viz.updatePose(...)` whenever there is a new pose, from any task. It never
  blocks.

Units are meters and radians. For inches and degrees use
`spectagatr::inchesToMeters` and `spectagatr::degreesToRadians`. Every call is
in the [Brain README](brain/README.md).

### 4. Run it

1. Upload and start the Brain program.
2. Plug the laptop into the V5 controller linked to the Brain, or straight into
   the Brain by USB.
3. Start the receiver:

```sh
build-spectagatr/receiver/spectagatr                                        # finds the controller, else the Brain
build-spectagatr/receiver/spectagatr --source controller --radio-download   # 10 Hz through the controller
```

The browser opens by itself. Ctrl+C stops the receiver. While it runs it holds
the controller's port, so stop it before `pros upload` or `pros terminal`.

## What the viewer shows

| Shown | Meaning |
|---|---|
| LIVE | Poses are arriving |
| STALE n s | No new pose for over 750 ms (`--stale-ms`). The robot stays dimmed at its last pose |
| Searching for VEX device | No controller or Brain found, or the link dropped |
| Receiver disconnected, retrying | The browser lost the receiver and reconnects by itself |

Unplugging and replugging needs no restart. The trail breaks on a new program
run, on `viz.resetTrail()`, after a stale gap and on a browser reconnect.

## Update rate

The controller radio sets the rate, not the code. Bench numbers from
2026-10-10, Brain sending 10 poses a second:

| Setup | Viewer updates |
|---|---|
| Pit channel (default) | About 3 a second, about 4 poses per burst |
| Download channel (`--radio-download`) | 10 a second, no loss |

- `--radio-download` moves the radio to the download channel and back to pit
  when the receiver stops. The Brain link drops for a few seconds while it
  switches, so switch with the robot disabled. Not yet checked: its effect on
  driving latency and whether it is legal under field control.
- On the pit channel, `c.rate_hz = 5` in the config avoids lost lines.
- Every `printf` in the program shares the same output. Heavy printing slows
  the pose lines.
- Only one program can read the Brain's output at a time.

## Field conventions

- Origin at the inside bottom left corner of the field diagram. +x right, +y up
  on the diagram.
- Heading from +x, counterclockwise positive. Heading 0 faces right on the
  diagram, not up the screen.
- Robot frame: +x forward, +y left, +z up.

Roll, pitch and the full table: [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Folders

| Folder | What |
|---|---|
| [`brain/`](brain/README.md) | Brain library: `spectagatr::Viewer` |
| [`protocol/`](protocol/include/spectagatr/protocol/pose_line.h) | Line format shared by the Brain and the receiver ([PROTOCOL.md](docs/PROTOCOL.md)) |
| [`receiver/`](receiver/README.md) | Laptop program: controller, Brain USB, sim and replay sources, web server, every option |
| [`web/`](web/README.md) | Browser viewer, works offline |
| [`fields/`](fields/README.md) | Field drawings: `override.json` (V5RC Override), `empty.json` |
| [`tools/`](tools/) | `mock_feed.py` (viewer without a receiver), `ws_probe.py` (feed checker) |

## Tests

```sh
cmake -S lib/spectaGATR -B build-spectagatr -G "MinGW Makefiles"
cmake --build build-spectagatr --parallel
ctest --test-dir build-spectagatr --output-on-failure
```

Host tests cover the line format, the Brain library core and the receiver.
Browser tests are in the [web README](web/README.md).

## Status and open items

- Run on hardware: the controller path once, 2026-10-10, with a simulated pose
  and before the robot and path lines existed.
- Not yet run on hardware: the robot and path lines, the Brain USB source, the
  receiver on Linux and macOS.
- Open: decide the competition setup (pit channel at about 3 Hz, or the
  download channel once its legality and driving latency are known).
- Open: STALE cannot tell "the Brain has no new pose" from "the link is slow".
  A flag from the Brain would fix that.
- If the receiver crashes (not Ctrl+C) while using `--radio-download`, the
  controller stays on the download channel until about 10 s after USB unplug.
