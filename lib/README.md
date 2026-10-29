# lib

Robot code reused across seasons. The control board link is in
GATR2-Electronics.

| Library | What |
|---|---|
| [locaGATR](locaGATR/README.md) | Robot pose: `getPose()`, `getX()` and so on |
| [naviGATR](naviGATR/README.md) | Autonomous routes: lines, Bezier curves, turns, actions |
| [spectaGATR](spectaGATR/README.md) | Live viewer: pose, size and path in a browser on the laptop |
| [gatr2/robot.h](gatr2/include/gatr2/robot.h) | The robot: ports, tracking wheels, drive. Shared by locaGATR and naviGATR |

## In a season project

[lib.mk](lib.mk), in the project `Makefile`:

```make
GATR2_LIB_ROOT ?= ../lib
GATR2_LIBS := spectagatr locagatr navigatr

# at the bottom, on both sides of common.mk
include $(GATR2_LIB_ROOT)/lib.mk
-include ./common.mk
include $(GATR2_LIB_ROOT)/lib.mk
```

## Adding a library

1. A folder here with its README.
2. `GATR2_LIB_SRC_<name>` and `GATR2_LIB_INC_<name>` in `lib.mk`.
3. A row in the table above.
