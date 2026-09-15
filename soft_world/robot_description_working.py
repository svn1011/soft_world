"""Generate the RViz description for the sensor-reconstructed soft limb."""

import math


DISK_COUNT = 6
DISK_PITCH = 0.025
DISK_RADIUS = 0.02
DISK_THICKNESS = 0.005
SENSOR_STATIONS = (
    (2, (0.0, 120.0, 240.0), "1 0.45 0 1"),
    (3, (180.0, 300.0, 60.0), "0 0.8 1 1"),
)


def _disk_link(index):
    sensors = []
    for disk_index, angles, color in SENSOR_STATIONS:
        if disk_index != index:
            continue
        for gauge_index, angle_degrees in enumerate(angles, start=1):
            angle = math.radians(angle_degrees)
            x = DISK_RADIUS * math.cos(angle)
            y = DISK_RADIUS * math.sin(angle)
            sensors.append(
                f"""
    <visual name="gauge_{gauge_index}">
      <origin xyz="{x:.8f} {y:.8f} 0" rpy="0 0 {angle:.8f}"/>
      <geometry><box size="0.001 0.004 0.005"/></geometry>
      <material name="sensor_{index}_{gauge_index}">
        <color rgba="{color}"/>
      </material>
    </visual>"""
            )

    disk_color = "0.08 0.2 0.8 1" if index == 0 else "0.7 0.7 0.72 1"
    return f"""
  <link name="soft_limb_disk_{index:02d}">
    <visual name="disk">
      <geometry>
        <cylinder radius="{DISK_RADIUS}" length="{DISK_THICKNESS}"/>
      </geometry>
      <material name="disk_{index:02d}"><color rgba="{disk_color}"/></material>
    </visual>{''.join(sensors)}
  </link>"""


def _interval(index):
    previous_disk = index - 1
    return f"""
  <link name="soft_limb_direction_frame_{index:02d}"/>
  <link name="soft_limb_bend_frame_{index:02d}"/>
  <joint name="soft_limb_direction_{index:02d}" type="revolute">
    <parent link="soft_limb_disk_{previous_disk:02d}"/>
    <child link="soft_limb_direction_frame_{index:02d}"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="1" velocity="10"/>
  </joint>
  <joint name="soft_limb_bend_{index:02d}" type="revolute">
    <parent link="soft_limb_direction_frame_{index:02d}"/>
    <child link="soft_limb_bend_frame_{index:02d}"/>
    <origin xyz="0 0 -{0.5 * DISK_PITCH}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="1" velocity="10"/>
  </joint>
  <joint name="soft_limb_untwist_{index:02d}" type="revolute">
    <parent link="soft_limb_bend_frame_{index:02d}"/>
    <child link="soft_limb_disk_{index:02d}"/>
    <origin xyz="0 0 -{0.5 * DISK_PITCH}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="1" velocity="10"/>
  </joint>"""


def build_robot_description():
    """Return a URDF using an exact symmetric circular-arc interval."""
    elements = [_disk_link(0)]
    for index in range(1, DISK_COUNT):
        elements.append(_interval(index))
        elements.append(_disk_link(index))
    return """<?xml version="1.0"?>
<robot name="soft_limb_sensor_follower">
  <material name="unused"><color rgba="0.7 0.7 0.72 1"/></material>
%s
</robot>
""" % "".join(elements)
