import subprocess

def spawn_marker(name, x, y, z=0.05):
    sdf = f"""
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <visual name='visual'>
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
        <material><ambient>0 1 0 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>
"""
    cmd = [
        "gz", "topic", "-t", "/world/empty/create", "-m", "gz.msgs.EntityFactory",
        "-p", f"sdf: \"{sdf}\", pose: {{ position: {{ x: {x}, y: {y}, z: {z} }} }}, name: \"{name}\""
    ]
    subprocess.run(cmd)

waypoints = [
    (-1.5, 1.3), (-1.5, 2.5), (0.5, 2.5), (1.5, 1.5), (0.5, 0.5), (-1.5, 0.5)
]

for i, (x, y) in enumerate(waypoints):
    spawn_marker(f"wp_marker_{i+1}", x, y)
