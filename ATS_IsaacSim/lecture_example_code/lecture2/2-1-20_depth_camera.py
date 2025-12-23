from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.sensor import Camera
import omni.replicator.core as rep
from omni.isaac.core import World
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np
import matplotlib.pyplot as plt
depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
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

camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.0, 0.0, 25.0]),
    frequency=20,
    resolution=(256  , 256  ),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
    
)

my_world.scene.add_default_ground_plane()
my_world.reset()

camera.initialize()

i = 0
camera.add_motion_vectors_to_frame()
camera.add_distance_to_camera_to_frame()
camera.get_current_frame()["distance_to_camera"]
while simulation_app.is_running():
    my_world.step(render=True)
    depth_image = camera.get_current_frame()["distance_to_camera"]
    if depth_image is not None and depth_image.size > 1:

        if i > 60:
            plt.imshow(depth_image, cmap="jet")  # RGB 값만 표시
            plt.axis("off")
            plt.show()
            i = 0
        else:
            pass
        i += 1

simulation_app.close()
