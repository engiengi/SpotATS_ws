from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})  # headless=False는 GUI를 띄움

from omni.isaac.core.utils.stage import open_stage
from isaacsim.storage.native import get_assets_root_path
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np


assets_root_path = get_assets_root_path()
open_stage(assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd")

my_world = World(stage_units_in_meters=1.0)


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
my_world.reset()

import omni.kit.commands
from pxr import Gf
import omni.replicator.core as rep

lidar_config = "Example_Rotary"
_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/World/sensor",
    parent=None,
    config=lidar_config,
    translation=(0, 0, 0.5),
    orientation=Gf.Quatd(1, 0, 0, 0),
)

render_product = rep.create.render_product(sensor.GetPath(), resolution=(512, 512))

annotator = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
annotator.attach(render_product)

writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
writer.attach(render_product)

simulation_app.update()
my_world.reset()

while simulation_app.is_running(): 
    simulation_app.update()
    data = annotator.get_data()
    print(data['distance'])

simulation_app.close()

