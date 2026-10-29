# spectaGATR protocol and conventions

One definition for the Brain publisher, the laptop receiver and the browser.
Version 1. Code: [`protocol/`](../protocol/include/spectagatr/protocol/pose_line.h).

## Coordinates and units

| Item | Convention |
|---|---|
| Field origin | Inside bottom left corner of the field diagram (the Override field.xml origin) |
| Field +X | Right on the field diagram |
| Field +Y | Up on the field diagram |
| Field +Z | Up out of the floor |
| Heading (yaw) | Angle of robot forward from field +X, counterclockwise positive. Heading 0 faces +X, which is right on the diagram, not up the screen |
| Robot frame | +x forward, +y left, +z up; origin is whatever point the caller's pose describes (usually the turning center) |
| Roll | About robot +x (forward); positive lifts the left side |
| Pitch | About robot +y (left); positive tips the nose down |
| Rotation | Robot to field: R = Rz(yaw) * Ry(pitch) * Rx(roll), intrinsic yaw, then pitch, then roll |
| API units | Meters and radians (`Meters`, `Radians`); helpers convert inches and degrees |
| Wire units | Millimeters and centidegrees, integers |

Attitude orients the robot. It never moves it: there is no robot height, and the
viewer never derives one from roll or pitch.

A heading-only update is the attitude {roll 0, pitch 0, yaw heading}, also after
an earlier full-attitude update.

## Lines (Brain to laptop)

Three line kinds share one frame: a five byte marker `SG?1 ` whose letter `?`
names the kind, single-space separated fields, `*`, a CRC and `\n`.

| Marker | Kind | Carries |
|---|---|---|
| `SGP1 ` | pose | one pose, the newest the Brain had |
| `SGR1 ` | robot | the footprint drawn around the pose point |
| `SGT1 ` | path point | one point of the planned path |

Common rules:

- session: 8 uppercase hex digits, nonzero, one per publisher run; the same on
  every kind of line that run sends.
- crc: 4 uppercase hex digits, CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) of
  every byte from `S` up to `*`.
- A line is at most 120 bytes and has no 0x00 byte, and the publisher writes one
  line per write. Each write is one PROS COBS packet when COBS is on (the PROS
  default: the stream id `sout` plus the line, with one code byte in front). With
  COBS off (another library can turn it off for all output), the bytes are raw.
  Either way the line appears verbatim in the byte stream, so the receiver scans
  for the marker and checks the CRC. It needs no COBS decoder. The same holds for
  the controller's USER_READ replies. The receiver drops trailing 0x00 bytes of
  each reply: the controller pads them (seen on the bench, 2026-10-10), and a
  padded reply would otherwise cut a line as truncated.
- Pose lines come first on the Brain. Robot and path lines go out only while the
  Brain's output FIFO is empty, so they never delay a pose by more than one short
  line, and they never carry anything the viewer needs for the pose itself.

### Pose line

```
SGP1 <session> <epoch> <seq> <t_ms> <x_mm> <y_mm> <yaw_cdeg> <roll_cdeg> <pitch_cdeg> <flags>*<crc>\n
```

| Field | Format | Meaning |
|---|---|---|
| epoch | decimal 0..65535 | Bumped by an explicit pose reset; the trail breaks |
| seq | decimal u32, wraps | +1 per pose line sent |
| t_ms | decimal u32, wraps | Publisher clock (Brain `millis`) when the pose was supplied |
| x_mm, y_mm | decimal i32, abs <= 2,000,000 | Field position |
| yaw, roll, pitch | decimal centidegrees in (-18000, 18000] | Attitude |
| flags | decimal u8 | bit 0: full attitude supplied (else roll = pitch = 0) |

Example: `SGP1 5A3C19E2 3 42 123456 1500 -250 9000 -150 75 1*XXXX`.

### Robot line

```
SGR1 <session> <front_mm> <back_mm> <left_mm> <right_mm> <height_mm>*<crc>\n
```

Distances from the pose point along robot +x, -x, +y, -y, and the height,
decimal millimeters 0..5000; the box must have a positive length, width and
height. Sent when the Brain program sets it, then again every 2 s, and again
under a new session. Example: `SGR1 5A3C19E2 170 230 150 150 250*XXXX`.

### Path point line

```
SGT1 <session> <path_id> <index> <count> <x_mm> <y_mm>*<crc>\n
```

| Field | Format | Meaning |
|---|---|---|
| path_id | decimal 0..65535 | +1 per `setPath` or `clearPath` on the Brain |
| index | decimal | 0..count-1 |
| count | decimal 0..65535 | Points in this path; 0 means no path (index, x, y are 0) |
| x_mm, y_mm | decimal i32 | Field position, as in the pose line |

The Brain sends every point once as fast as its output allows, then repeats the
whole path about every 2 s (never faster than one line per 50 ms), so a lost line
is filled in on the next pass. The Brain keeps at most 256 points. Example:
`SGT1 5A3C19E2 7 3 12 -250 1500*XXXX`.

### Receiver rules

| Case | Rule |
|---|---|
| Partial reads, several lines per read | `LineScanner` keeps state across chunks, for every kind |
| Console text between lines | Skipped; the marker may follow other text on the same line |
| 0x00 or a new marker before `\n` | Line dropped as truncated |
| Line over 120 bytes, control bytes | Dropped and counted |
| Bad CRC, malformed, out of range, non-finite | Dropped and counted, whatever the kind |
| New session id, on any kind of line | Publisher restart: the trail breaks, ordering restarts, the robot and path of the old run are dropped (the new run sends its own) |
| Same session, new epoch | Pose reset: the trail breaks |
| Same session and epoch, seq not newer (`int32(seq - last) <= 0`) | Repeated or out of order: dropped and counted |
| Same session and epoch, seq more than 64 behind | A restarted publisher whose best-effort session id collided: a restart. A serial stream does not reorder lines |
| t_ms | Only compared within one session, modulo 2^32; never against a laptop clock |
| Freshness | The receiver stamps each accepted pose with its own monotonic clock. The age shown is the local receive age, not the end-to-end latency, which is not measured |
| Robot line | The newest wins; the same box again changes nothing |
| Path point | Kept by index under its (session, path_id); a new id, a new session or a different count starts the path over; a repeated point changes nothing. The receiver keeps at most 4096 points |

## Receiver to browser

HTTP and WebSocket on `http://127.0.0.1:8790/` by default.

| Path | Content |
|---|---|
| `/` and static files | `web/` |
| `/fields/<name>.json` | Field definitions |
| `/api/fields` | `{"fields": ["override", ...], "default": "override"}` |
| `/ws` | WebSocket, one JSON object per text frame, receiver to browser only |

`hello`, sent once on connect. `robot` is the receiver's default box
(`--robot`), shown until a `robot` message replaces it:

```json
{"type": "hello", "protocol": 1, "receiver": "spectagatr 1.0", "stale_ms": 750,
 "field": "override", "fields": ["override", "empty"],
 "robot": {"front_m": 0.1685, "back_m": 0.1685, "left_m": 0.1465, "right_m": 0.1465,
           "height_m": 0.25}}
```

`pose`, once per accepted pose, newest only: a slow browser gets the latest
pose, never a backlog:

```json
{"type": "pose", "session": "5A3C19E2", "epoch": 3, "seq": 42, "t_ms": 123456,
 "x": 1.5, "y": -0.25, "yaw": 1.5708, "roll": -0.0262, "pitch": 0.0131,
 "attitude": true, "restart": false, "rx_ms": 98765.4, "source": "controller"}
```

- Values are SI: meters and radians.
- `restart` is true on the first pose of a new session or epoch.
- `rx_ms` is the receiver's monotonic clock in milliseconds.

`robot`, when the Brain's footprint changes, and after `hello` on connect when
one is known. `session` is null when a new run has not sent one yet; the box is
then the receiver's default again:

```json
{"type": "robot", "session": "5A3C19E2", "front_m": 0.22, "back_m": 0.16,
 "left_m": 0.18, "right_m": 0.18, "height_m": 0.3}
```

`path`, the whole path as assembled so far, at most once per receiver tick
(about 50 ms) while it changes, newest only; after `hello` on connect when one
is known. A point not received yet is null. `count` 0 is a path the Brain
cleared; `session` and `id` null mean no path at all:

```json
{"type": "path", "session": "5A3C19E2", "id": 7, "count": 3, "known": 2,
 "points": [[1.5, -0.25], null, [0, 3.566]]}
```

`status`, on any change and at least every 250 ms:

```json
{"type": "status", "rx_ms": 98800.0, "source": "controller", "port": "COM7",
 "link": "open", "detail": "controller: radio linked", "stale": false,
 "pose_age_ms": 35,
 "counts": {"poses": 1234, "bad_crc": 0, "malformed": 0, "out_of_range": 0,
            "repeated": 0, "out_of_order": 0, "truncated": 0, "too_long": 0,
            "garbage": 0, "sessions": 1, "bytes": 56789,
            "robot_lines": 3, "path_points": 120}}
```

| Field | Values |
|---|---|
| `source` | `"controller"`, `"brain"`, `"sim"`, `"file"` (replay) or `"none"` |
| `port` | Serial port name or replay path; `""` when there is none |
| `link` | `"open"`, `"searching"` or `"error"` |
| `pose_age_ms` | Receiver-local age of the newest accepted pose; null before the first |
| `stale` | True when there is no pose, the age exceeds `stale_ms`, or the link is not open |
| `counts.robot_lines`, `counts.path_points` | Accepted robot and path lines; corrupt ones count with the pose drops |

The browser also marks everything disconnected when the WebSocket closes, and
reconnects by itself.
