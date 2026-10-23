W, D, H, T = 20.0, 12.0, 3.0, 0.2   # room width, depth, height, wall thickness (m)
WIN, SILL, TOP = 2.0, 1.0, 2.2      # window width, bottom edge, top edge (m)
parts = []
def box(name, x, y, z, sx, sy, sz, color="0.8 0.8 0.8 1"):
    parts.append(f"""
      <link name="{name}">
        <pose>{x} {y} {z} 0 0 0</pose>
        <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><diffuse>{color}</diffuse><ambient>{color}</ambient></material></visual>
      </link>""")
def wall_with_window(name, cx, cy, length, along_x):
    side = (length - WIN) / 2
    off = WIN / 2 + side / 2
    for i, s in enumerate([-1, 1]):
        x, y = (cx + s * off, cy) if along_x else (cx, cy + s * off)
        sx, sy = (side, T) if along_x else (T, side)
        box(f"{name}_side{i}", x, y, H / 2, sx, sy, H)
    sx, sy = (WIN, T) if along_x else (T, WIN)
    box(f"{name}_sill", cx, cy, SILL / 2, sx, sy, SILL)
    box(f"{name}_lintel", cx, cy, (TOP + H) / 2, sx, sy, H - TOP)
wall_with_window("east",  W / 2, 0, D, along_x=False)   # morning sun
wall_with_window("south", 0, -D / 2, W, along_x=True)   # midday sun
wall_with_window("west", -W / 2, 0, D, along_x=False)   # afternoon sun
box("north", 0, D / 2, H / 2, W + T, T, H)
box("roof", 0, 0, H + 0.05, W + T, D + T, 0.1, "0.5 0.5 0.5 1")
for i, (x, y) in enumerate([(-4, 2), (3, 3), (5, -2), (-6, -3)]):
    box(f"pallet{i}", x, y, 0.5, 1.2, 1.0, 1.0, "0.6 0.4 0.2 1")
world = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="warehouse">
    <!-- 4 ms step (Gazebo default is 1 ms): /clock is published every step, and at 1 kHz the
         Python nodes on sim time (sun_sensor, sun_brain, plant_monitor) used ~80 % CPU each -->
    <physics name="4ms" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>
    <!-- Lighting is only for looking at the scene. The simulated sun (sun_sensor, RViz) cannot
         be shown by a Gazebo light: Fortress cannot move lights at runtime. So this lamp casts
         no shadows (no fake, fixed "sun patch" through a window) and the room is lit evenly. -->
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.6 0.75 0.9 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="room_light">
      <cast_shadows>false</cast_shadows>
      <direction>-0.3 0.2 -1.0</direction>
      <diffuse>0.55 0.55 0.55 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
    </light>
    <model name="ground"><static>true</static><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></visual>
    </link></model>
    <model name="warehouse"><static>true</static>{''.join(parts)}
    </model>
  </world>
</sdf>"""
import os
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "warehouse.sdf")
open(out, "w").write(world)
print(out, "created")
