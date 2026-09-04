# SoftWorld

SoftWorld is a ROS 2 Humble package for simulating a constant-curvature,
multi-disc soft limb in NVIDIA Isaac Sim. The active **working** configuration
contains one physical leader limb in Isaac Sim, six simulated strain gauges,
and an RViz model reconstructed from the live sensor readings.

The previous configuration with both leader and follower limbs in Isaac Sim is
retained as a backup.

## System overview

```text
Position or torque command
          |
          v
Isaac Sim leader limb
          |
          v
Six pose-based strain gauges
          |
          v
Curvature reconstruction
          |
          v
/joint_states -> robot_state_publisher -> /tf -> RViz limb
```

The RViz limb does not receive the leader's command messages. Its shape is
calculated from sensor measurements, so moving the Isaac Sim limb manually
while simulation is running also updates the RViz model.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- NVIDIA Isaac Sim 4.5.0 (location configured with `ISAAC_SIM_PATH`)
- A working NVIDIA driver and display
- `colcon`
- ROS packages `rviz2`, `robot_state_publisher`, `tf2_ros`, `sensor_msgs`,
  and `std_msgs`
- Python 3 Matplotlib with the Tk backend

For a standard ROS 2 Humble installation, source:

```text
/opt/ros/humble/setup.bash
```

If ROS was built from source, replace that path with the `setup.bash` in that
ROS installation. Do not copy another user's absolute home-directory path.

Set the Isaac Sim installation directory before launching. For example:

```bash
export ISAAC_SIM_PATH="$HOME/isaacsim"
```

The working launch defaults to `$HOME/isaacsim` when this variable is omitted.

For a standard binary ROS 2 Humble installation, install the additional
packages with:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-matplotlib \
  python3-tk ros-humble-rviz2 ros-humble-robot-state-publisher \
  ros-humble-tf2-ros ros-humble-sensor-msgs ros-humble-std-msgs
```

## Build

### Preparing the ZIP archive

The sender should create the archive from the directory containing the
`soft_world` folder. Do not include generated ROS build products because they
can contain absolute paths and symlinks from the original computer:

```bash
zip -r soft_world.zip soft_world \
  -x 'soft_world/build/*' \
     'soft_world/install/*' \
     'soft_world/log/*' \
     'soft_world/**/__pycache__/*' \
     'soft_world/.pytest_cache/*'
```

The source scripts, launch files, Python package, USD assets, `package.xml`,
`setup.py`, and this README must remain in the archive.

### Extracting and building

Place `soft_world.zip` in the directory where the project should live, then
extract it. The archive should produce a folder named `soft_world`:

```bash
unzip soft_world.zip
cd soft_world
export SOFT_WORLD_WS="$PWD"
```

`SOFT_WORLD_WS` records the extracted location without assuming a user name
or home-directory layout. Then build the package:

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
colcon build --packages-select soft_world
source install/setup.bash
```

Source both setup files in every new terminal:

```bash
source /opt/ros/humble/setup.bash
export SOFT_WORLD_WS="/path/where/you/extracted/soft_world"
cd "$SOFT_WORLD_WS"
source install/setup.bash
```

## Launch the working system

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ISAAC_SIM_PATH="$HOME/isaacsim"
ros2 launch soft_world soft_world_working.launch.py
```

This starts:

- the leader limb in Isaac Sim;
- `robot_state_publisher` with the generated limb description;
- the static transform `map -> soft_limb_disk_00`;
- RViz2 with incompatible Snap paths removed from its environment;
- the Matplotlib window containing all six live strain-gauge graphs.

The limb starts with zero bend targets. Start a controller only after the
simulation has loaded.

## Display the limb in RViz

The working launch opens RViz automatically.

1. Expand **Global Options** and set **Fixed Frame** to `map`.
2. Click **Add** at the bottom of the Displays panel.
3. Select **RobotModel** and click **OK**.
4. Expand the new **RobotModel** display.
5. Confirm **Description Source** is `Topic`.
6. On **Description Topic**, double-click the value cell on the right and
   enter `/robot_description`.
7. Ensure **Visual Enabled** is checked.

The Description Topic row can be expanded to show QoS settings, but expanding
it does not edit the topic. If its value cell is hidden, drag the divider
between the property and value columns toward the left.

If the model is outside the view, select RobotModel, press `F`, and zoom with
the mouse wheel. The expected model is a downward chain of 12 discs with a
blue base, gray body, orange one-third gauges, and cyan two-thirds gauges.
In Isaac Sim the free final disc is red. The RViz reconstruction currently
renders that final disc gray; this color difference is not a tracking error.

## Position control

In a second terminal:

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/control_soft_limb_working.py --ros-args \
  -p phi_position:=10.0 \
  -p theta_position:=90.0
```

- `phi_position`: total bend magnitude in degrees.
- `theta_position`: bend direction around the limb in degrees.

The total bend is distributed over 11 disc intervals. Each dual-D6 A/B joint
receives half of its interval angle. The requested angle stays as the PhysX
drive target while the acceleration drive generates mass-dependent corrective
torque from position and velocity error. Targets ramp at no more than 20°/s to
avoid exciting flexible modes. Change parameters while running with:

```bash
ros2 param set /soft_limb_position_controller phi_position 15.0
ros2 param set /soft_limb_position_controller theta_position 180.0
```

Run only one soft-limb controller at a time.

The current source default is `phi_position=180.0` and
`theta_position=0.0`. Supplying explicit command-line parameters is therefore
recommended. The hybrid controller defaults to zero torque and zero direction.

Commanded phi is the orientation change between the base and tip discs, not
the angle of the straight base-to-tip chord. For an ideal 90° circular arc,
the tip disc rotates 90° while that chord lies at approximately 45°.

Verify the physical response with:

```bash
ros2 topic echo /soft_limb/measured_tip/phi
ros2 topic echo /soft_limb/measured_tip/theta
ros2 topic echo /soft_limb/measured_tip/orientation
```

### Current runtime tuning

| Parameter | Active value |
|---|---:|
| Physics frequency | 240 Hz |
| Solver position/velocity iterations | 32 / 8 |
| Base drive stiffness/damping | 6400 / 240 |
| Split A/B drive stiffness/damping | 12800 / 480 |
| Body linear/angular damping | 2.0 / 10.0 |
| Target ramp limit | 20°/s |
| Per-joint X/Y bend limit | ±25° |
| Torque command limit | ±1,000,000 N·m |
| Torque slew limit | 2,000,000 N·m/s |

Each moving disc and virtual link has mass `0.01 kg`. In position mode the
loader applies an upward force of `mass * 9.81 m/s²` to every moving body to
remove gravity-induced bend error. This compensation switches off whenever
nonzero direct torque control is active.

Tune the actively launched system in `scripts/load_soft_limb_working.py`, then
rebuild. At startup this loader overrides drive values stored in the USD. The
constants in the USD-generator scripts affect newly generated assets, but do
not replace the loader's runtime override.

## Hybrid torque/position control (PENDING)

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/control_soft_limb_hybrid_working.py --ros-args \
  -p phi_torque:=1000.0 \
  -p theta_position:=90.0
```

- `phi_torque`: bending torque in N·m.
- `theta_position`: bend direction in degrees.

Torque mode controls effort and therefore does not guarantee an exact angle.

## Constant-curvature convention

Theta chooses the bending plane; it is not physical twist. Z twist is locked.
The controller uses the Webster–Jones convention:

```text
rotX = -phi * sin(theta)
rotY =  phi * cos(theta)
```

Theta wraps around the circumference. For example, 0°, 90°, 180°, and 270°
select four transverse bending directions.

## ROS 2 interface summary

The supplied control scripts publish at 20 Hz. The simulator node
`/soft_limb_command_subscriber` consumes:

| Joint group | Position mode | Hybrid mode |
|---|---|---|
| Base | `/soft_limb/pair_00/base/roll/position`, `/yaw/position` | `/soft_limb/pair_00/base/roll/torque`, `/yaw/position` |
| A half, pairs 01–10 | `/soft_limb/pair_XX/d6_a/roll/position`, `/yaw/position` | `/soft_limb/pair_XX/d6_a/roll/torque`, `/yaw/position` |
| B half, pairs 01–10 | `/soft_limb/pair_XX/d6_b/roll/position`, `/yaw/position` | `/soft_limb/pair_XX/d6_b/roll/torque`, `/yaw/position` |

Normal operation should use the supplied controller parameters rather than
publishing all of these low-level topics individually.

## Simulated strain gauges

| Station | Disc | Circumferential positions | Color |
|---|---:|---|---|
| `one_third` | `Disk_04` | 0°, 120°, 240° | Orange |
| `two_thirds` | `Disk_07` | 180°, 300°, 60° | Cyan |

The second ring is offset 180° from the first. Since each ring repeats every
120°, this is visible geometrically as a 60° stagger.

Readings are calculated from live disc poses, not control targets. They respond
to position control, torque control, gravity/contact, and manual manipulation.

### Sensor topics

```text
/soft_limb/sensors/<station>/strain
/soft_limb/sensors/<station>/orientation
/soft_limb/sensors/<station>/phi
/soft_limb/sensors/<station>/theta
/soft_limb/sensors/<station>/gauge_1/strain
/soft_limb/sensors/<station>/gauge_2/strain
/soft_limb/sensors/<station>/gauge_3/strain
```

- `strain`: three-element `Float64MultiArray` in microstrain.
- `orientation`: relative quaternion `[w, x, y, z]` from the base.
- `phi`: measured bend magnitude in degrees.
- `theta`: measured bend direction in degrees.

Inspect the streams with:

```bash
ros2 topic echo /soft_limb/sensors/one_third/strain
ros2 topic echo /soft_limb/sensors/two_thirds/orientation
ros2 topic echo /joint_states --once
ros2 topic echo /robot_description --once
ros2 topic echo /tf --once
```

### Live strain graphs

The working launch starts `plot_strain_gauges_working.py`, which displays six
independent plots in a two-by-three window. The top row is the one-third ring,
the bottom row is the two-thirds ring, the horizontal axis is a rolling
20-second window, and the vertical axis is microstrain (µε).

To run only the plotter manually after starting the simulation:

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/plot_strain_gauges_working.py
```

The microstrain is a kinematic emulation derived from live disc poses and the
assumed 20 mm gauge radius. It is suitable for reconstruction and controller
testing, but is not a calibrated prediction for a particular physical gauge,
adhesive layer, or limb material.

### RViz reconstruction

The two gauge rings reconstruct local X/Y curvature. Curvature is interpolated
over 11 intervals. Each RViz interval uses an exact symmetric circular-arc
transform: direction rotation, half-spacing translation, bend rotation,
half-spacing translation, and inverse direction rotation. This uses 33 RViz
joints and makes the extended radial disc lines share a common curvature
center when the measured curvature is constant. Arbitrary shapes between the
two stations are still approximated because two measurement sections cannot
uniquely determine every possible deformation along the limb.

## In-simulation follower backup

The earlier physical follower is preserved in:

```text
scripts/load_soft_limb_in_simulation_working.py
launch/soft_world_in_simulation_working.launch.py
```

Run the backup with:

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch soft_world soft_world_in_simulation_working.launch.py
```

## Troubleshooting

### RViz reports a GLIBC or `libpthread` symbol error

Snap-installed VS Code can inject incompatible Core20 GTK/GIO paths. The
working launch removes them automatically. For manual RViz startup use:

```bash
env -u SNAP \
  -u SNAP_LIBRARY_PATH \
  -u GTK_PATH \
  -u GIO_MODULE_DIR \
  XDG_DATA_DIRS=/usr/local/share:/usr/share:/var/lib/snapd/desktop \
  rviz2
```

Warnings about `canberra-gtk-module` are harmless.

### RViz says `Frame [map] does not exist`

Restart the working launch. It starts `soft_limb_map_transform`. Verify it:

```bash
ros2 run tf2_ros tf2_echo map soft_limb_disk_00
```

### RobotModel does not appear

Verify RViz uses `map` and `/robot_description`, then check:

```bash
ros2 topic echo /robot_description --once
ros2 topic echo /joint_states --once
ros2 topic echo /tf --once
```

### Strain plots are missing or empty

If the window does not open, check the launch terminal for a Tk or Matplotlib
error and install `python3-matplotlib python3-tk`. If it opens without data,
make sure Isaac Sim is playing and check both streams:

```bash
ros2 topic hz /soft_limb/sensors/one_third/strain
ros2 topic hz /soft_limb/sensors/two_thirds/strain
```

### Limb bends without starting a controller

A controller can survive a closed VS Code window and continue publishing its
last command. Inspect active nodes and processes:

```bash
ros2 node list
pgrep -af 'control_soft_limb'
```

Normally there should be no `soft_limb_position_controller` or
`soft_limb_hybrid_controller` before you start one. Stop only the confirmed
stale PID:

```bash
kill <PID>
```

### Launch cannot find a script

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
colcon build --packages-select soft_world
source install/setup.bash
```

### Python cannot import Isaac Sim modules

Run simulation and USD generator scripts through
`$ISAAC_SIM_PATH/python.sh`. The normal system Python does not include modules
such as `pxr` and `isaacsim`.

## Soft quadruped

The older quadruped implementation is separate from the active RViz workflow:

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch soft_world soft_quadraped.launch.py
```

In another terminal:

```bash
python3 scripts/control_quadraped.py
```

## Project layout

### Active working files

- `launch/soft_world_working.launch.py` — Isaac Sim, robot-state publisher,
  map transform, and sanitized RViz launch.
- `scripts/load_soft_limb_working.py` — leader physics, command subscriber,
  sensors, curvature reconstruction, and `/joint_states`.
- `scripts/control_soft_limb_working.py` — position controller.
- `scripts/control_soft_limb_hybrid_working.py` — hybrid controller.
- `scripts/plot_strain_gauges_working.py` — six live microstrain graphs.
- `soft_world/robot_description_working.py` — generated 12-disc RViz URDF.
- `usd/soft_limb_hanging_working.usd` — active hanging asset.
- `usd/soft_limb_position_control_working.usd` — working position asset.
- `scripts/soft_limb_hang_working.py` and `scripts/soft_limb_working.py` — USD
  generators.

### Backup files

- `scripts/load_soft_limb_in_simulation_working.py` — two-limb Isaac version.
- `launch/soft_world_in_simulation_working.launch.py` — backup launch.

Files without `_working` are preserved older versions. Quadruped files are
currently independent of the active working limb.

## Safe shutdown

Stop the controller first with `Ctrl+C`, then stop the launch with `Ctrl+C`.
Closing VS Code may leave terminal processes running. After a forced close:

```bash
ros2 node list
pgrep -af 'control_soft_limb|load_soft_limb|isaacsim'
```

Terminate only a confirmed stale PID.

## Working-file policy

Current development changes belong in files containing `_working`. Files
without `_working` remain unchanged unless shared package infrastructure must
be updated explicitly.
