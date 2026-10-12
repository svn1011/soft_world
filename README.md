# SoftWorld

SoftWorld is a ROS 2 Humble package for simulating constant-curvature,
multi-disc soft limbs in NVIDIA Isaac Sim. The **Data-Collection** branch uses
the `soft_3_working` robot: three hanging six-disc modules with independent
torque and bending-direction commands and 18 simulated strain gauges.

The repository also retains the single-module RViz workflow and the older
in-simulation follower configuration as backups.

## Data-collection system overview

```text
Manual three-module controller     Automated 61-case collector
               \                   /
                v                 v
             ROS torque/theta commands
                        |
                        v
         Three-module soft robot in Isaac Sim
                        |
                        v
             18 virtual strain gauges
                        |
                        v
       Live voltage plots + timestamped XLSX data
```

The automated collector commands the robot, waits until applied torque and
theta reach their targets, and then records one synchronized 18-sensor
snapshot. The manual controller and collector are alternative command sources,
not processes to run together.

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

## Data-Collection branch: quick start

Use `soft_3_working` for both manual robot control and automated data
collection. Start the simulator first, and then choose **either** manual control
**or** data collection. Do not run the controller and collector together
because both publish commands to the same robot.

### Terminal 1 — start the three-module robot

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ISAAC_SIM_PATH="$HOME/isaacsim"
ros2 launch soft_world soft_3_working.launch.py
```

Wait until Isaac Sim has loaded the robot. This launch also opens the live
graphs for all 18 bridge-voltage sensors.

### Terminal 2A — manually control the robot

Open another terminal and run:

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/control_soft_3_working.py --ros-args \
  -p module_1_phi_torque:=0.50 -p module_1_theta_position:=0.0 \
  -p module_2_phi_torque:=0.75 -p module_2_theta_position:=90.0 \
  -p module_3_phi_torque:=1.00 -p module_3_theta_position:=-90.0
```

For each module, `phi_torque` is the bending torque in N m and
`theta_position` is the bending direction in degrees. Torque is limited to
plus or minus 1.0 N m. Press `Ctrl+C` in this terminal to stop the controller.

### Terminal 2B — collect the complete voltage dataset

First stop `control_soft_3_working.py` with `Ctrl+C` if it is running. Keep the
simulator from Terminal 1 open, then run in a second sourced terminal:

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/collect_soft_3_working.py
```

The collector controls the robot automatically, so no separate control script
is needed. The default run performs 61 torque/theta cases and records exactly
one snapshot per successful case after the applied torque and theta reach
their commands. A successful complete run produces 61 synchronized samples
and 1,098 sensor rows.

The new workbook is saved under `research_data_working/` with a timestamped
name such as `Final_Data_collection_working_YYYYMMDD_HHMMSS.xlsx`. It contains
`Scenario Information`, `Sensor Map`, `Samples`, and `Final Sensor Data`.
The protected `research_data_working/Final Data.xlsx` reference is never
overwritten; the collector also rejects any existing `--output` path.

In short:

```text
Manual test:       launch simulator -> run control_soft_3_working.py
Dataset collection: launch simulator -> run collect_soft_3_working.py
Never run the controller and collector at the same time.
```

## Single-module RViz system (optional)

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

The six-disc working limb now has a 125 mm base-to-tip centerline: five
25 mm intervals, with a 20 mm clear gap between 5 mm-thick discs. Runtime
base stiffness is 6400 and damping 240; split A/B joint gains remain twice
the base values. Body damping,
gravity compensation, solver settings, sensor topics, and graphing are retained.
The original twelve-disc USDs remain unchanged for backup; the launch loads
the dedicated six-disc hanging USD directly, without runtime disc removal.

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
the mouse wheel. The expected model is a downward chain of 6 discs with a
blue base, gray body, orange one-third gauges, and cyan two-thirds gauges.
In Isaac Sim the free final disc is red. The RViz reconstruction currently
renders that final disc gray; this color difference is not a tracking error.

## Position control

For the six-disc limb, use **`control_soft_limb_working.py`**, not
`control_soft_limb.py`. The original controller is a preserved twelve-disc
backup: it divides phi by 11 and previously produced only about 81.8° on five
intervals for a 180° request. The working controller divides by 5: base 36°,
then four serial A/B pairs at 18° per joint. The final (sixth) disc is
`Disk_05`, because numbering starts at zero. Stop old controllers before
starting the working controller; do not run both together.

The working loader now also recognizes the original position controller's
complete twelve-disc publisher topology and converts its joint bend targets
by 11/5 before applying the six-disc limits. A warning reports this conversion.
It does not rescale the six-disc working controller, manual joint commands,
or hybrid torque. If duplicate controllers share the same node name, conversion
is ambiguous: stop the duplicates rather than relying on automatic conversion.
After changing loader code, rebuild with `colcon build --packages-select soft_world`,
source `install/setup.bash` in both terminals, and restart the simulator.

From the extracted repository root, after rebuilding and sourcing the install:

```bash
ros2 launch soft_world soft_world_working.launch.py
```

In another sourced terminal, from that same repository root:

```bash
python3 scripts/control_soft_limb_working.py --ros-args \
  -p phi_position:=180.0 -p theta_position:=0.0
```

The working loader rejects assets that do not contain exactly six discs,
and uses an explicitly ordered six-disc view and explicit `Disk_05` tip.

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

The total bend is distributed over 5 disc intervals. Each dual-D6 A/B joint
receives half of its interval angle. The requested angle stays as the PhysX
drive target while the acceleration drive generates mass-dependent corrective
torque from position and velocity error. Targets ramp at no more than 30°/s to
avoid exciting flexible modes. Because each later physical joint combines its
A and B compatibility commands, each half ramps at 15°/s; their sum therefore
matches the base joint's 30°/s rate. All five physical intervals consequently
advance together instead of bending segment by segment. Change parameters
while running with:

```bash
ros2 param set /soft_limb_position_controller phi_position 15.0
ros2 param set /soft_limb_position_controller theta_position 180.0
```

Run only one soft-limb controller at a time.

The current position source default is `phi_position=90.0` and
`theta_position=0.0`. Supplying explicit command-line parameters is therefore
recommended. The hybrid script defaults to `0.0 N·m`, so it does not actuate
the limb automatically; pass `phi_torque` explicitly for a reproducible
experiment.

The position controller accepts total phi up to ±180°. At 180° the five
intervals each receive 36°: the base D6 receives 36°, and each later serial
A/B joint receives 18°. Physical X/Y joint limits are ±80° to accommodate
the extended torque-admittance range. Larger position requests are clamped;
theta still selects directions around the full circumference.

Commanded phi is the orientation change between the base and tip discs, not
the angle of the straight base-to-tip chord. For an ideal 90° circular arc,
the tip disc rotates 90° while that chord lies at approximately 45°.

Verify the physical response with:

```bash
ros2 topic echo /soft_limb/measured_tip/phi
ros2 topic echo /soft_limb/measured_tip/theta
ros2 topic echo /soft_limb/measured_tip/orientation
```
## Hybrid torque/direction control (calibrated admittance)

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/control_soft_limb_hybrid_working.py --ros-args \
  -p phi_torque:=0.5 \
  -p theta_position:=90.0
```

- `phi_torque`: torque-command input in N·m for the calibrated admittance.
- `theta_position`: bend direction in degrees.

The working hybrid controller is a calibrated torque-to-curvature admittance
controller. It accepts torque in N·m and maps the requested quasi-static
equilibrium bend linearly:

```text
total phi = 180 degrees * torque / 1.0 N*m
```

Consequently, 0.1, 0.5, and 1.0 N·m request approximately 18°, 90°, and
180°. Negative torque reverses the bend. The command is clamped to ±1.0 N·m,
so larger magnitudes cannot request a bend beyond 180°. This is a 100×
rescaling of the command interface; the simulated
equilibrium behavior is unchanged. The range corresponds to approximately
0--10.2 kgf·cm and is easier to compare with compact actuator ratings, but it
is not a measured torque requirement for the physical limb.
Headless Isaac regressions measured 18.0000°, 90.0000°, and 179.9999° at
those three commands, with equal bend across all five intervals.
An explicit 1.5 N·m over-limit regression was clipped to 1.0 N·m and measured
179.9999°, confirming that the active controller cannot request more than 180°.

Measured phi is now the sum of the local bend vectors between adjacent discs.
A single base-to-tip quaternion uses the shortest rotation and would fold
angles above 180° back toward zero. The active controller is now intentionally
limited to 180°.

This is not raw open-loop torque actuation and it is not a physical
torque-versus-angle calibration. The acceleration drives enforce the uniform
constant-curvature equilibrium derived from the torque command. This behavior
was selected because raw body torque depended strongly on gravity and joint
inertia and did not reproduce the position-controlled constant-curvature shape.
The former 1000 N·m example was also far beyond the useful range and forced
joints against their limits. The USD still contains the historical serial A/B
joints for compatibility, but the loader deactivates those constraints and
creates one direct D6 joint per disc interval before PhysX initializes. The
loader vector-sums the two legacy A/B half-angle commands into that direct
joint's full interval target. This removes the serial pair's former
52° ceiling. A prior development configuration demonstrated a repeatable 270°
result, but the active working controller now stops at 180°. The virtual helper
links are retained as kinematic compatibility objects.

While a nonzero torque command is active, bend-position callbacks are ignored;
this prevents a stale position controller from cancelling torque at 20 Hz.
The loader warns once when it rejects such a position command. Publish zero
torque before returning to position control. Running one controller at a time
is still required for reproducible experiments.

## Constant-curvature convention

Theta chooses the bending plane; it is not physical twist. Z twist is locked.
The controller uses the Webster–Jones convention:

```text
rotX = -phi * sin(theta)
rotY =  phi * cos(theta)
```

Theta wraps around the circumference. For example, 0°, 90°, 180°, and 270°
select four transverse bending directions.
This two-axis drive mapping is approximate at large diagonal bends. A headless
180° test at theta=0 measured 179.9996° with a steady final angle; at theta=45°
it measured about 178.93° but retained about 10.25° of full tip-orientation error.
Exact large-angle diagonal constant-curvature tracking remains a limitation.

## ROS 2 interface summary

The supplied control scripts publish at 20 Hz. The simulator node
`/soft_limb_command_subscriber` consumes:

| Joint group | Position mode | Hybrid mode |
|---|---|---|
| Base | `/soft_limb/pair_00/base/roll/position`, `/yaw/position` | `/soft_limb/pair_00/base/roll/torque`, `/yaw/position` |
| A half, pairs 01–04 | `/soft_limb/pair_XX/d6_a/roll/position`, `/yaw/position` | `/soft_limb/pair_XX/d6_a/roll/torque`, `/yaw/position` |
| B half, pairs 01–04 | `/soft_limb/pair_XX/d6_b/roll/position`, `/yaw/position` | `/soft_limb/pair_XX/d6_b/roll/torque`, `/yaw/position` |

Normal operation should use the supplied controller parameters rather than
publishing all of these low-level topics individually.

## Simulated strain gauges

| Station | Disc | Circumferential positions | Color |
|---|---:|---|---|
| `one_third` | `Disk_02` | 0°, 120°, 240° | Orange |
| `two_thirds` | `Disk_03` | 180°, 300°, 60° | Cyan |

The second ring is offset 180° from the first. Since each ring repeats every
120°, this is visible geometrically as a 60° stagger.

For six discs, the exact thirds fall between discs. These rings are on the
nearest symmetric pair at 40% and 60% of the base-to-tip centerline. Topic
station names are preserved for compatibility.

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
/soft_limb/sensors/<station>/bridge_voltage
/soft_limb/sensors/<station>/bridge_valid
/soft_limb/sensors/<station>/gauge_1/bridge_voltage
/soft_limb/sensors/<station>/gauge_1/bridge_valid
```

- `strain`: three-element `Float64MultiArray` in microstrain.
- `orientation`: relative quaternion `[w, x, y, z]` from the base.
- `phi`: measured bend magnitude in degrees.
- `theta`: measured bend direction in degrees.
- `bridge_voltage`: three ideal quarter-bridge outputs in volts.
- `bridge_valid`: three values, where `1.0` is within the configured foil-gauge
  and DAQ range and `0.0` is outside it. Individual gauge validity is also
  available as a `Bool` topic.

Inspect the streams with:

```bash
ros2 topic echo /soft_limb/sensors/one_third/strain
ros2 topic echo /soft_limb/sensors/two_thirds/orientation
ros2 topic echo /soft_limb/sensors/one_third/bridge_voltage
ros2 topic echo /soft_limb/sensors/one_third/bridge_valid
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
material, or adhesive system.

### Virtual Wheatstone-bridge voltage

Each of the six existing virtual gauges now also feeds an ideal independent
quarter-Wheatstone bridge. The current research assumptions are a 350 ohm foil
gauge, nominal gauge factor 2.0, and 2.5 V excitation. For the simulated
surface strain `epsilon`, the output is:

```text
Vout = -Vexc * (GF * epsilon) / (4 + 2 * GF * epsilon)
```

The sign depends on which bridge arm the physical gauge occupies; this working
convention must be retained or changed consistently in hardware and ROS. A
second six-graph window opens with output volts. It plots the ideal circuit
value even when the reading is invalid; inspect the `bridge_valid` topic before
interpreting a value as hardware-feasible.

At compressive strain of $-50\%$, the ideal active-gauge resistance reaches
zero and the ROS bridge-voltage topic becomes `NaN`; this is intentionally not
presented as a valid electrical measurement. To prevent Matplotlib from making
the trace disappear, the voltage graph alone displays non-finite samples at
the dashed +1.25 V (`Vexc/2`) invalid-model boundary and logs a warning. The
underlying ROS value remains `NaN`, and `bridge_valid` remains false. Voltage
plots use 0.02 V minimum padding instead of the strain plot's 1.0-unit padding,
so finite changes between 1.0 and 1.25 V remain visible.

`bridge_valid=1.0` currently requires both `abs(epsilon) <= 3%` and
`abs(Vout/Vexc) <= 25 mV/V`. These are deliberately conservative nominal foil
gauge/DAQ assumptions, not a purchased part's calibration. The current limb's
high-bend virtual strains can exceed this range, so its voltage output is a
virtual circuit model until a physical limb material, mounting layer, selected
gauge, and DAQ are validated.
adhesive layer, or limb material.

### RViz reconstruction

The two gauge rings reconstruct local X/Y curvature. Curvature is interpolated
over 5 intervals. Each RViz interval uses an exact symmetric circular-arc
transform: direction rotation, half-spacing translation, bend rotation,
half-spacing translation, and inverse direction rotation. This uses 15 RViz
joints and makes the extended radial disc lines share a common curvature
center when the measured curvature is constant. Arbitrary shapes between the
two stations are still approximated because two measurement sections cannot
uniquely determine every possible deformation along the limb.

## Three-module hanging snake (`soft_3_working`)

The next working configuration places three six-disc modules in series. It has
18 physical discs, 15 actuated bending intervals, two rigid inter-module
connectors, three independent phi-torque inputs, three independent theta
directions, and 18 virtual strain gauges (six per module). The first disc is
fixed in the air, so the complete chain hangs under gravity.

Build and launch it from the extracted repository root:

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
colcon build --packages-select soft_world
source install/setup.bash
export ISAAC_SIM_PATH="$HOME/isaacsim"
ros2 launch soft_world soft_3_working.launch.py
```

This launch starts both Isaac Sim and the Matplotlib window containing all 18
live bridge-voltage graphs. The plotter may appear before Isaac Sim finishes
loading; its traces begin updating as soon as the simulator publishes sensor
data. Do not start a second voltage plotter unless a separate window is
intended.

In a second sourced terminal, publish one torque and direction pair for each
module. For example:

```bash
python3 scripts/control_soft_3_working.py --ros-args \
  -p module_1_phi_torque:=0.50 -p module_1_theta_position:=0.0 \
  -p module_2_phi_torque:=0.75 -p module_2_theta_position:=90.0 \
  -p module_3_phi_torque:=1.00 -p module_3_theta_position:=-90.0
```

Each module uses the same active single-module mapping,
`phi = 180 degrees * torque / 1.0 N*m`, with torque limited to plus or minus
1.0 N*m and bend magnitude limited to 180 degrees per module. The five
interval targets inside a module advance together. Theta selects its bending
direction independently of the other two modules.

Commands use these topic pairs:

```text
/soft_snake/module_1/phi_torque
/soft_snake/module_1/theta_position
/soft_snake/module_2/phi_torque
/soft_snake/module_2/theta_position
/soft_snake/module_3/phi_torque
/soft_snake/module_3/theta_position
```

Measured module bend and direction are available as
`/soft_snake/module_N/measured_phi` and
`/soft_snake/module_N/measured_theta`. Sensor topics use
`/soft_snake/module_N/sensors/{one_third,two_thirds}/gauge_G/` followed by
`strain`, `bridge_voltage`, or `bridge_valid`, where `N` and `G` are 1--3.
This gives 18 channels for each reported sensor quantity.

Open all 18 strain plots in another sourced terminal:

```bash
python3 scripts/plot_soft_3_working.py
```

To open an additional bridge-voltage plot window manually:

```bash
SOFT_SNAKE_PLOT_SIGNAL=bridge_voltage \
  python3 scripts/plot_soft_3_working.py
```

The headless three-module regression can be repeated with:

```bash
"$ISAAC_SIM_PATH/python.sh" scripts/verify_soft_3_working.py \
  --torques 0.5 0.75 1.0 --thetas 0 90 -90 --seconds 12
```

The verified run measured 90.0047, 134.9999, and 179.9999 degrees for the
three modules, respectively, reported all 18 sensor publishers, and passed
the synchronized-interval and rigid-connector checks. These are simulation
results for the implemented admittance law, not hardware torque calibration.

The generated asset is `usd/soft_3_working.usd`. Regenerate it only through
Isaac Sim's Python environment:

```bash
"$ISAAC_SIM_PATH/python.sh" scripts/soft_3_working.py
```

The three-module loader selects Isaac Sim's **Grey Studio** viewport light rig
and uses the same rig for documented simulation screenshots.

### Collect torque/theta voltage data in Excel

Keep the simulator launch running, but stop
`control_soft_3_working.py` so two nodes do not publish competing commands.
Then run the automated collector from a second sourced terminal:

```bash
cd "$SOFT_WORLD_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/collect_soft_3_working.py
```

The default `one_at_a_time` experiment records a zero-torque baseline followed
by four torques (0.25, 0.50, 0.75, and 1.00 N m) at five theta directions
(-90, -45, 0, 45, and 90 degrees) for each of the three modules. This is 61
cases. Each case records exactly one synchronized snapshot only after the
simulator's applied torque is within 0.005 N m of its command and applied theta
is within 2 degrees of its command continuously for 0.5 seconds. A case that
does not reach both commands within 15 seconds is marked as failed and produces
no voltage row. The exact case count, gate result, wait time, progress, and
destination are printed during collection. Commands return to zero when the run
finishes or is interrupted with `Ctrl+C`.

The default one-iteration experiment is:

```bash
python3 scripts/collect_soft_3_working.py \
  --torques 0.25 0.50 0.75 1.00 \
  --thetas -90 -45 0 45 90 \
  --repetitions 1 \
  --settle-seconds 3 \
  --samples-per-case 1 \
  --torque-tolerance-nm 0.005 \
  --theta-tolerance-deg 2 \
  --target-hold-seconds 0.5 \
  --target-timeout 15
```

This produces 61 synchronized snapshots and 1,098 long-format sensor rows when
all targets succeed. Use `--sweep-mode all_same` to apply each torque/theta pair
to all three modules at once. Use `--output path/name.xlsx` to choose a filename.
Without it, the timestamped workbook is saved under `research_data_working/`.

Each new workbook matches the read-only `Final Data.xlsx` structure and contains
`Scenario Information`, `Sensor Map`, `Samples`, and `Final Sensor Data`.
`Final Sensor Data` is the primary analysis-ready table: each row is one sensor
observation, and its first seven columns are `case number`, `module number`,
`section number`, `sensor number`, `torque command Nm`, `theta command deg`,
and `voltage V`. `section number` is `1/3` or `2/3`, `sensor number` is
`G1`, `G2`, or `G3`,
and torque/theta are the exact commands for that sensor's module. Additional
columns preserve applied torque, applied theta, commanded and measured phi,
circumferential sensor angle, and sensor ID. This is the same 13-column schema
used by `Final Data.xlsx`.

The 38-column `Samples` sheet includes all three
commanded torque/theta pairs, all three measured phi/theta pairs, all 18 bridge
voltages, and all three applied torque/theta pairs. The
collector refuses to run
beside the normal controller unless the deliberate
`--allow-competing-controller` override is supplied. These voltages are still
pose-derived simulation outputs from the ideal bridge model; an Excel export
does not make them hardware-calibrated measurements.

`research_data_working/Final Data.xlsx` is the protected reference dataset.
The collector never overwrites an existing workbook. New runs default to a
timestamped `Final_Data_collection_working_*.xlsx` file, and `--output` must
also name a path that does not already exist.

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
- `soft_world/robot_description_working.py` — generated 6-disc RViz URDF.
- `usd/soft_limb_hanging_6_disc_working.usd` — active six-disc hanging asset.
- `usd/soft_limb_position_control_6_disc_working.usd` — six-disc position asset.
- `scripts/create_six_disc_assets_working.py` — regenerates both variants using
  Isaac Sim's fully initialized native generators (requires a working GPU).
- `scripts/verify_bend_working.py` — bounded headless physical bend/stability
  verification, using the active loader's drives and compensation.
- `scripts/update_six_disc_geometry_working.py` — updates authored native
  six-disc spacing, anchors, and limits using a USD-enabled interpreter.
- `scripts/soft_limb_hang_working.py` and `scripts/soft_limb_working.py` — USD
  generators.
- `usd/soft_3_working.usd` — active three-module hanging snake asset.
- `launch/soft_3_working.launch.py` — three-module simulator
  launch.
- `scripts/load_soft_3_working.py` — three-module physics,
  independent control, measured bends, and 18-sensor publication.
- `scripts/control_soft_3_working.py` — three independent
  phi-torque/theta command pairs.
- `scripts/plot_soft_3_working.py` — 18 strain or bridge-voltage
  plots.
- `scripts/verify_soft_3_working.py` — bounded headless
  three-module regression.
- `scripts/collect_soft_3_working.py` — configurable torque/theta sweep and
  18-channel Excel data collection.
- `scripts/xlsx_writer_working.py` — dependency-free XLSX workbook writer.
- `scripts/test_xlsx_writer_working.py` — offline XLSX structure regression.
- `scripts/soft_3_working.py` — `soft_3_working.usd`
  generator.

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
