#!/usr/bin/env python3
import random
import subprocess
import tempfile
import os

COLORS = {
    'red':   (0.9, 0.1, 0.1),
    'blue':  (0.1, 0.1, 0.9),
    'green': (0.1, 0.8, 0.1),
}

def main():
    color_name = random.choice(list(COLORS.keys()))
    r, g, b = COLORS[color_name]
    print(f'[spawn_trash] Spawning {color_name} trash at (2.2, 0.0)')

    sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="trash">
    <static>true</static>
    <pose>2.0 0.0 0.25 0 0 0</pose>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>0.3 0.3 0.5</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

    with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as f:
        f.write(sdf)
        tmp = f.name

    subprocess.run([
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-file', tmp,
        '-name', 'trash',
        '-x', '2.0',
        '-y', '0.0',
        '-z', '0.25',
    ])
    os.unlink(tmp)

if __name__ == '__main__':
    main()
