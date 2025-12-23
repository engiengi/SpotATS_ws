from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.prims import XFormPrim


my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
my_world.reset()

from omni.isaac.core.objects import DynamicCuboid
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np

cube = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube",
        name="cube",
        position=np.array([0.0, 0.0, 1.0]),
        orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
        scale=np.array([0.5, 0.5, 0.5]),
        size=1.0,
        color=np.array([255, 0, 0]),
    )
)
while simulation_app.is_running():
    my_world.step(render=True)

simulation_app.close()
