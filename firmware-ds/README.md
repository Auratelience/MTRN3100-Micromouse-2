# Domain-Specific Firmware (firmware-ds)

Used for testing specific robot parameters that do not require the full firmware.

Bring-up sketches. `lidar/lidar.ino` is the only one: it initialises the I2C bus
and the three VL6180Xs and prints their ranges to the serial monitor, forever.

```sh
arduino-cli compile --fqbn arduino:renesas_uno:nanor4 lidar
arduino-cli upload  --fqbn arduino:renesas_uno:nanor4 -p /dev/ttyACM0 lidar
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=9600
```

```
F: 142
L: 61
R: 400
```

Front, left, right, in mm, at loop rate. `400` is the ceiling — the sensors run
at `L::X2` scaling (2 mm resolution, 400 mm range) and an out-of-range return is
clamped rather than reported as an error.

## What it is for

Nothing here plans, drives or estimates anything. It is the sketch to reach for
when a range looks wrong on the robot and you want to know whether the problem
is the sensor, the bus, or everything downstream of them:

* all three sensors were found and re-addressed at boot (`0x30`/`0x31`/`0x32`),
* a beam pointed at a wall reads roughly the distance to it,
* readings survive more than a few seconds — a bus that wedges shows up here as
  frozen numbers, and `I2CRepairer` recovering shows up as a pause and a
  resumption.

Setup prints `[OKAY]` per stage over serial at 9600 baud.

## Contents

`lidar/` is a full Arduino sketch directory, so it carries a **copy** of every
header from `firmware/micromouse/` — the toolchain requires headers to sit
beside the `.ino`, and there is no include path to point elsewhere. The sketch
itself only uses five of them:

| used | not used, but present |
| --- | --- |
| `constants.h`, `pins.h`, `i2cRepairer.h`, `lidar.h`, `types.h` | `control.h`, `imu.h`, `kinematics.h`, `motor.h`, `observers.h`, `oled.h`, `planners.h`, `sensorFusion.h` |

There is no `maze_map.h` or `maze_path.h` here; nothing localises.

## Keeping it in sync

Copies drift. Today every header here is byte-identical to its counterpart in
`firmware/micromouse/` **except one**:

```sh
diff -rq firmware/micromouse firmware-ds/lidar    # from the repo root
```

`sensorFusion.h` still initialises `xWeightTotal`, `yWeightTotal` and
`dthetaWeightTotal` to `1.0f`, where `firmware/micromouse/` now starts them at
`0.0f`. At `1.0f` the dead-reckoning model's vote is a vote for the *origin*
rather than for where dead reckoning actually says the robot is, so any pose
source drags the estimate toward (0, 0) — the defect written up in
[`firmware-sim/README.md`](../firmware-sim/README.md). It has no effect on this
sketch, which never constructs a `SensorFusion`, but do not copy this file back
over the firmware's.

Run the `diff` before trusting anything you conclude here about the robot.
