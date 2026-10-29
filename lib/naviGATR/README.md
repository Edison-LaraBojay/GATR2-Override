# naviGATR

Autonomous routes and path following, using the pose from a locaGATR
Localizer. Inches and degrees.

```cpp
const navigatr::Route route = navigatr::Route({24, 12, 90})   // x, y, heading
    .bezier({36, 24}, {48, 48}, {60, 48}).facing(0)
    .line({60, 72})
    .at(0.5, [] { intake.move(127); })
    .turnTo(180)
    .wait(300);

localizer.setPose(route.startPose());
navigator.follow(route);
```

## Setup

1. locaGATR set up first.
2. Season `Makefile`: `GATR2_LIBS := spectagatr locagatr navigatr`.
3. Limits and gains in `src/navigation.cpp`, then:

   ```cpp
   navigatr::Navigator navigator(config(), localizer);
   ```

## Routes

| Call | Does |
|---|---|
| `Route({x, y, heading})` | Start pose |
| `line({x, y})` | Straight to the point |
| `bezier(c1, c2, end)` | Cubic Bezier: two control points, then the end |
| `facing(heading)` | Last segment ends facing this way |
| `tangent()` | Last segment faces along the path |
| `at(fraction, action)` | Action at a fraction of the last segment, or at the start of a turn or wait |
| `turnTo(heading)` | Turn in place |
| `wait(ms)` | Stop |

Lines and curves in a row are one move, no stop between them. The heading
holds unless `facing()` or `tangent()` says otherwise. Actions run in the
navigator task, so keep them short.

## Running

| Call | Does |
|---|---|
| `follow(route)` | Runs it, blocks. True if every step settled |
| `start(route)` | Runs it, returns at once |
| `wait()` | Until done |
| `waitUntil(fraction)` | Until that fraction of the driving distance |
| `cancel()` | Stops the route and the drive |
| `isRunning()`, `progress()` | |
| `outcome()` | Cancelled, steps timed out, first one |

```cpp
navigator.start(route);
navigator.waitUntil(0.5);
lift.score();  // slow work while it drives
navigator.wait();
```

A route stops on `cancel()` or when the competition mode changes. A step
settles within `tolerances` for `settle_ms`, or gives up `timeout_ms` late;
the rest of the route still runs.

## Pipeline

The whole route is planned first, then each move runs in `Pipeline::run()`
([pipeline.cpp](src/pipeline.cpp)):

```
1. path      segments -> Path
2. profile   Path -> Trajectory          speed and time per point
then every tick:
3. follow    pose + target -> command    forward, left, turn
4. drive     command -> motors
```

| Stage | Implementations |
|---|---|
| Profile | `TrapezoidProfile` (default): slower in tight curves and fast heading changes |
| Follower | `HolonomicFollower` (default): path velocity plus PID on x, y, heading |
| Drive | `MecanumDrive` |

`Config` ([config.h](include/navigatr/config.h)) picks the implementations and
holds the limits, gains, tolerances and the shared `gatr2::Robot`.

## Tuning

Start around 24 in/s with the localizer already checked.

| Setting | How |
|---|---|
| `Robot::drive` | Measure rpm, gear ratio, wheel diameter, track width, wheelbase. `kv` comes from these |
| `feedforward.ks` | Raise until the robot just moves |
| `feedforward.kv` | Leave at 0 unless it runs consistently fast or slow |
| `gains.kp`, `gains.heading_kp` | Raise until it holds the path without wobble |
| `limits` | Raise once it follows cleanly |

## Host tests

From GATR2-Override (Windows: MSYS2 ucrt64 on PATH, `-G "MinGW Makefiles"`):

```sh
cmake -S lib/naviGATR -B build-navigatr
cmake --build build-navigatr --parallel
ctest --test-dir build-navigatr --output-on-failure
```

## Status

33 host tests, including whole routes on a simulated robot. Builds with PROS
4.2.2. Not run on a robot; the navigator task (start, wait, cancel) has not
run at all.
