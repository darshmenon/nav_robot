#!/usr/bin/env python3
"""Single source of truth: generates worlds/office.sdf, maps/office.{pgm,yaml} and keepout/speed masks."""
import os
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RES, HALF = 0.05, 6.0          # map covers [-6, 6] m
# (name, cx, cy, sx, sy)
BOXES = [
    ("wall_n", 0, 5.1, 10.4, 0.2), ("wall_s", 0, -5.1, 10.4, 0.2),
    ("wall_e", 5.1, 0, 0.2, 10.0), ("wall_w", -5.1, 0, 0.2, 10.0),
    ("inner_a", -1.5, -3.0, 0.2, 4.0),      # doorway between y=-1 and y=1.5
    ("inner_b", -1.5, 3.25, 0.2, 3.5),
    ("inner_c", 3.0, 1.5, 4.0, 0.2),
    ("pillar_1", 2.0, -2.0, 0.8, 0.8),
    ("pillar_2", -3.5, 2.5, 0.6, 0.6),
    ("pillar_3", 0.8, 3.5, 0.5, 0.5),
]
KEEPOUT = (2.5, 5.0, -4.5, -1.0)   # x0, x1, y0, y1 (no-go zone)
SLOW = (-1.0, 1.5, 1.0, 4.0)       # x0, x1, y0, y1 (speed-limited zone) -> doorway/hall

def box_sdf(n, cx, cy, sx, sy):
    return f"""    <model name="{n}"><static>true</static><pose>{cx} {cy} 0.5 0 0 0</pose>
      <link name="l"><collision name="c"><geometry><box><size>{sx} {sy} 1</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>{sx} {sy} 1</size></box></geometry>
      <material><ambient>0.7 0.7 0.75 1</ambient><diffuse>0.7 0.7 0.75 1</diffuse></material></visual></link></model>
"""

def zone_sdf(n, z, rgba):
    x0, x1, y0, y1 = z
    return f"""    <model name="{n}"><static>true</static><pose>{(x0+x1)/2} {(y0+y1)/2} 0.005 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>{x1-x0} {y1-y0} 0.01</size></box></geometry>
      <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual></link></model>
"""

def write_world():
    body = "".join(box_sdf(*b) for b in BOXES)
    body += zone_sdf("keepout_zone", KEEPOUT, "0.9 0.1 0.1 1")
    body += zone_sdf("slow_zone", SLOW, "0.95 0.8 0.1 1")
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="office">
    <physics name="1ms" type="ode"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.3 0.2 -1</direction></light>
    <model name="ground"><static>true</static><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
      <material><ambient>0.35 0.4 0.35 1</ambient><diffuse>0.35 0.4 0.35 1</diffuse></material></visual></link></model>
{body}  </world>
</sdf>
"""
    open(f"{ROOT}/worlds/office.sdf", "w").write(sdf)

def grid():
    n = int(2 * HALF / RES)
    return n, np.full((n, n), 254, np.uint8)

def paint(img, x0, x1, y0, y1, val):
    n = img.shape[0]
    c0, c1 = int((x0 + HALF) / RES), int((x1 + HALF) / RES)
    r0, r1 = n - int((y1 + HALF) / RES), n - int((y0 + HALF) / RES)
    img[max(r0, 0):r1, max(c0, 0):c1] = val

def save(name, img, mode="trinary", extra=""):
    Image.fromarray(img).save(f"{ROOT}/maps/{name}.pgm")
    open(f"{ROOT}/maps/{name}.yaml", "w").write(
        f"image: {name}.pgm\nmode: {mode}\nresolution: {RES}\norigin: [{-HALF}, {-HALF}, 0.0]\n"
        f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

def write_maps():
    n, img = grid()
    for _, cx, cy, sx, sy in BOXES:
        paint(img, cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, 0)
    save("office", img)
    _, k = grid(); paint(k, *KEEPOUT, 0); save("keepout_mask", k)
    # speed mask: value encodes percentage (raw 0..100 scaled by 255) in 'scale' mode
    _, s = grid(); s[:] = 255; paint(s, *SLOW, 128)
    save("speed_mask", s, mode="scale")

if __name__ == "__main__":
    write_world(); write_maps(); print("generated world + maps")
