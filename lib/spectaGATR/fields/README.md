# spectaGATR fields

Field definitions the viewer draws. The receiver serves each `<name>.json` here
at `/fields/<name>.json` and lists the names at `/api/fields`. The viewer loads
only these JSON files at runtime; nothing comes from the network or the Brain.

| File | Field |
|---|---|
| `override.json` | V5RC Override 2026-2027 nominal geometry and colors |
| `empty.json` | Plain 12 ft (3.6576 m) square, 2 ft tiles, no objects |

To add a field, write a new `<name>.json` by hand following the schema below
and drop it in this folder; the receiver lists it on the next start. The
Override numbers came from Game Manual v2.0 Appendix A and the official CAD and
are nominal.

## Schema `spectagatr-field/1`

Meters and degrees. Field frame as in `../docs/PROTOCOL.md`: origin at the
inside bottom-left corner of the field diagram, +x right, +y up on the diagram,
+z up, angles counterclockwise from +x.

| Key | Meaning |
|---|---|
| `schema` | `"spectagatr-field/1"` |
| `name` | File name without `.json` |
| `title`, `source` | Display text |
| `dimensions` | `inside_x_m`, `inside_y_m` (inside wall faces, from the origin), `wall_height_m`, `wall_thickness_m`, `tile_m` (grid pitch; 0 for none) |
| `walls` | Optional labels: `x_min`, `x_max`, `y_min`, `y_max`, each `{"label", "color"}` |
| `objects` | List of objects below |

Object kinds (all have `id`, `kind`, `x_m`, `y_m`, `yaw_deg`, `color` `"#rrggbb"`):

| `kind` | Extra keys | Drawn as |
|---|---|---|
| `box` | `z_m`, `size_x_m`, `size_y_m`, `size_z_m` | Box centered at (x, y, z), `size_x_m` along its own x after `yaw_deg` |
| `tape` | `size_x_m` (length), `size_y_m` (width) | Flat strip on the floor centered at (x, y) |
| `octagon` | `height_m`, `base_across_flats_m`, `top_across_flats_m` | Tapered octagonal prism standing on the floor at (x, y), flats on its axes |

Geometry here is display data only. Override values are nominal and provisional
(see the notes in the source XML).
