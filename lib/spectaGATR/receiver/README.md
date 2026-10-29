# spectagatr receiver

Laptop program between the robot and the browser viewer. It reads pose, robot footprint and path
point lines (PROTOCOL.md, `SGP1`, `SGR1`, `SGT1`) from a V5 controller, a Brain USB port, the
built-in sim or a replay, and serves the viewer (`web/`), the field definitions (`fields/`) and a
WebSocket feed on `http://127.0.0.1:8790/`. Structure: [PIPELINE.md](PIPELINE.md).

## Build and run

```
cmake -S lib/spectaGATR -B build-spectagatr -G "MinGW Makefiles" # from GATR2-Override; Windows, ucrt64 GCC
cmake --build build-spectagatr --parallel
ctest --test-dir build-spectagatr --output-on-failure
build-spectagatr/receiver/spectagatr --source sim                 # opens the browser
cmake --install build-spectagatr --prefix C:/spectagatr           # bin/, share/spectagatr/
```

Each build copies `web/` and `fields/*.json` next to the program. The program looks for them
there, then in `../share/spectagatr/`, then in the source tree. On MinGW it links statically, so
it runs without the ucrt64 DLLs on PATH. Nothing is fetched at runtime.

## Options

| Option | Default | Meaning |
|---|---|---|
| `--source auto\|controller\|brain\|sim\|file` | auto | where poses come from |
| `--port NAME` | search | this serial port (COM7, /dev/ttyACM1) |
| `--list-ports` | | print USB serial ports with their VEX role, then exit |
| `--debug-link` | off | once a second to stderr: controller transport counters and pose delivery |
| `--user-read-length N` | 0 | controller USER_READ length byte (0 to 255); 0 returns up to 224 bytes per reply, 64 at most 64 |
| `--radio-download` | off | move a radio-linked controller to the download channel, back to pit on exit |
| `--sim figure8\|circle\|square\|attitude\|restart\|dropout` | figure8 | sim motion (implies `--source sim`) |
| `--rate HZ` / `--seed N` | 10 / 1 | sim rate (1..50) and session seed |
| `--replay PATH` (`-` = stdin) | | replay raw bytes (implies `--source file`) |
| `--replay-rate-bps N` / `--loop` | 12800 / off | replay pace; start the file again at its end |
| `--http-port N` / `--bind ADDR` | 8790 / 127.0.0.1 | viewer address |
| `--field NAME` | override | field shown first (`fields/<name>.json`) |
| `--web DIR` / `--fields DIR` | next to the program | viewer files, field definitions |
| `--stale-ms MS` | 750 | pose older than this is stale |
| `--robot F,B,L,R[,H]` | 0.1685,0.1685,0.1465,0.1465,0.25 | robot box in meters, shown until the Brain sends its own footprint |
| `--no-browser` | | only print the URL |

Ctrl+C stops it.

For intermittent controller delivery, run `spectagatr --source controller --debug-link`. Two
lines a second go to stderr:

- `controller ...`: cumulative poll, empty-poll, timeout, bad-reply, late-reply, stray-byte,
  padding-byte and delivered-byte counters, channel switches, the last round trip in ms and the
  radio channel (`--` when unknown; 5 is download per vexide).
- `delivery ...`: for the last second, poses accepted and their rate, the longest gap between
  poses, lines lost in transport (seq gaps), gaps over `--stale-ms`, lag growth (poses arriving
  older than the freshest ever seen, a sign of a queue), source bytes per second, lines the
  parser dropped (`corrupt`), replies that carried data and poses per such reply.

The radio channel sets the controller's update rate (bench, 2026-10-10, publisher at 10 Hz):

- Pit channel, the default: each USER_READ costs about 235 ms of dead time plus about 0.7 ms per
  byte. Length 0 (the default, as vexide) returns up to 224 bytes per reply, about 520 B/s,
  against about 200 B/s with pros-cli's 64. Replies come about 3 times a second carrying about
  4 poses each, and the viewer shows the newest, so the screen updates about 3 times a second.
  Two reads outstanding only doubled the round trip: the controller serves one at a time.
- Download channel, `--radio-download`, as `pros terminal` does: 30 to 50 ms round trips, 10 Hz
  arriving one pose per reply with gaps of 110 to 180 ms and no loss. The Brain link drops for
  a few seconds while the radio moves, so switch with the robot disabled. Its effect on joystick
  latency while driving and its legality under field control are unverified. The controller
  goes back to pit when the receiver stops (Ctrl+C), or by itself about 10 s after USB unplug.

The Brain publisher writes only while its output queue is short, so a slow link gives fewer but
fresh poses, never a growing backlog.

## Sources

- **auto**: a V5 controller if one is plugged in, else a Brain user port. Searches about once a
  second until one appears.
- **controller**: CDC2 polling of the controller's port. It sends SYSTEM_VERSION, USER_READ
  (channel 1) and RADIO_STATUS (every 10 s): never stdin bytes, and a radio channel switch only
  with `--radio-download`.
  `status.detail` shows the link ("controller: Brain linked by radio, radio quality 90" or
  "controller: no Brain linked"). The RADIO_STATUS layout comes from vexide; its channel field
  matched the bench (5 on download, 30 to 32 on pit). Trailing NUL bytes of each reply are
  dropped: the controller pads replies (seen on the bench), and a NUL inside a line would drop
  it as truncated.
- **brain**: the Brain's USB user port, read raw. It never writes to it: stdin bytes like
  `pRc` change PROS serial settings for the whole program.
- **sim**: deterministic motion inside the field, sent as real pose lines through the same
  scanner, decoder and tracker. `attitude` stands still and sweeps yaw +90, roll +20 (left side
  up), pitch +20 (nose down) degrees, one at a time, to check axis signs. `restart` starts a new
  session every 15 s with a pose reset (epoch) between. `dropout` goes silent 3 s of every 10 s.
- **file**: bytes from a file or stdin at `--replay-rate-bps` (paced by elapsed time), whatever
  they hold (raw text, PROS COBS packets). Link `open` while bytes flow, `searching` at the end.
  `--loop` rewinds a file and starts ordering over.

## Port selection

Roles come from USB ids, never from port names or COM numbers: Brain 2888:0501 interface 2 is
the user port, interface 0 the system port (uploads; never opened), controller 2888:0503. Two
controllers or two Brains are refused with both listed; pick one with `--port`. A `--port` that
the listing does not know is read raw like a Brain port (read-only either way).

## Reconnect

On an unplug or read error the link shows `error` with the reason, then `searching`, and the
same or another device is opened again, without restarting the program. A line cut by the
disconnect is dropped, not joined to the next bytes. If the Brain program kept running, its
poses continue (same session, newer seq) and the trail goes on; a new Brain program run (new
session) or a pose reset (new epoch) breaks the trail. The browser reconnects by itself.

Late or split controller replies are still parsed. A physical reconnect resets the scanner so
bytes from different connections cannot join. STALE (default 750 ms) means the pose is not
fresh, not necessarily that USB or the radio dropped.

## Status, stale and age

- `pose_age_ms` is how long ago this receiver accepted the newest pose, on its own monotonic
  clock. It is the local receive age, not the end-to-end latency from the Brain's measurement
  to the screen, which is not measured. The Brain's `t_ms` is passed on, never compared with
  laptop time.
- `stale` is true with no pose yet, an age over `--stale-ms`, or a link that is not `open`.
- `counts` has every drop reason: bad_crc, malformed, out_of_range, repeated, out_of_order,
  truncated, too_long, garbage.
- A slow browser never builds a queue: each gets the newest pose and status (a restart pose is
  kept in order), and a client that stops reading for 5 s is closed.

## Limitations

- One host reads the Brain's stdout. Polling the controller while another program holds the
  Brain's USB port is untested and may lose bytes to it.
- Platforms: Windows built and tested here. The controller source ran against a real controller
  and Brain on 2026-10-10; the Brain USB source is tested against fakes only. Linux (x86_64) and
  macOS (arm64) were only cross-compiled (zig, clang 18; macOS against a hand-written IOKit
  declaration shim, not the SDK), never run.
