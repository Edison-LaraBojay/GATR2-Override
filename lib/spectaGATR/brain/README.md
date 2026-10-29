# spectaGATR Brain library

`spectagatr::Viewer` sends the robot's pose, its size and the planned path from
a PROS program to the laptop viewer. Setup and a full example program are in
the [spectaGATR README](../README.md).

| File | What |
|---|---|
| `include/spectagatr/viewer.h` | The API. Source in `pros/`, PROS builds only |
| `include/spectagatr/pose.h` | `Meters`, `Radians`, `Attitude`, `Point`, `RobotBox`, unit helpers |
| `include/spectagatr/publisher.h`, `src/` | The portable core under the API, host tested |
| `tests/` | Host tests |

## Calls

| Call | What it does |
|---|---|
| `Viewer viz;` or `Viewer viz(config);` | One global. Config: `rate_hz` (pose lines per second, 1 to 50, default 10) and `robot` |
| `start()` | From `initialize()`. The first call creates the background task; later calls resume output |
| `stop()` | Pauses output |
| `updatePose(x, y, heading)` | The newest pose |
| `updatePose(x, y, Attitude{roll, pitch, yaw})` | The newest pose with roll and pitch |
| `resetTrail()` | After re-placing the robot or resetting odometry; the viewer starts a new trail |
| `setPath(points)` | `std::vector<Point>`, `std::vector<std::array<double, 2>>` or a pointer and count. At most 256 points |
| `clearPath()` | Removes the path |
| `setRobot(RobotBox)` | A new robot size while running |
| `setRate(hz)`, `rate()`, `stats()`, `session()` | Pose rate, counters, this run's id |

`RobotBox::centered(length, width, height)` is a box centered on the pose
point. For an off-center pose point use
`RobotBox{front, back, left, right, height}`, each measured from that point. A
box of all zeros sends nothing and the viewer keeps its default box.

## Rules

- Every call is safe from any task and never blocks. `setPath` from one task
  at a time.
- Meters and radians. Units are never guessed; convert with `inchesToMeters`
  and `degreesToRadians`.
- Bad input is dropped and counted in `stats()`, and the previous value stays:
  NaN or infinity, x or y over 2000 m, a box over 5 m or with no size, a path
  with a bad point.
- A heading-only `updatePose` sends roll and pitch as 0, even after an earlier
  call with an `Attitude`.

## How it sends

- One background task, priority `TASK_PRIORITY_DEFAULT - 1`, checks every 5 ms.
- It sends a pose line when there is a new pose, the rate period has passed and
  the Brain's output has room (at most about two lines waiting). A slow reader
  gets fewer lines, each with the newest pose, never a backlog. With no reader
  it stops sending.
- Robot and path lines go only while the output is empty, so a pose waits at
  most one short line behind them. The robot size is sent at once, then every
  2 s. A new path goes point by point, then is sent again about every 2 s so a
  lost point is filled in.
- Each line is one `write()` to stdout, one PROS COBS packet, 45 to 88 bytes.
  `pros terminal` and the receiver both see whole lines. Your own `printf`
  output can share stdout; the receiver skips anything that is not a
  spectaGATR line, but heavy printing delays poses.
- The library never reads stdin and never changes serial settings.
- Fixed memory after construction.

## Tests

Built with the rest of spectaGATR (see the [main README](../README.md#tests)).
Only the Brain library's tests:

```sh
ctest --test-dir build-spectagatr -R "^Specta"
```

On a Brain: a simulated pose went through a V5 controller to the receiver on
2026-10-10. The current `Viewer` builds with the PROS 4.2.2 toolchain with 0
warnings and has not run on a Brain yet.
