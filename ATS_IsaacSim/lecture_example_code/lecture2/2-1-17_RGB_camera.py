from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.sensor import Camera
from omni.isaac.core import World
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np
import matplotlib.pyplot as plt
my_world = World(stage_units_in_meters=1.0)
cube = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube",
        name="cube",
        position=np.array([0.0, 0.0, 1.0]),
        orientation=[0, 0, 0, 1],
        scale=np.array([0.5, 0.5, 0.5]),
        size=1.0,
        color=np.array([255, 0, 0]),
    )
)
camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.0, 0.0, 25.0]), # 카메라를 위쪽 상공 25m에 위치시켜 전체 장면을 내려다 보게 함
    frequency=60, # 초당 60프레임으로 영상 수집. 실시간 응답성이 필요한 경우 사용
    resolution=(256  , 256  ), # 출력 이미지 크기. 작게 설정하면 빠르게 실행되며 메모리 사용량도 줄어듦
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
    
)

my_world.scene.add_default_ground_plane()
my_world.reset()
camera.initialize()
i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    print(camera.get_current_frame())
    rgba_image = camera.get_rgba()

    if rgba_image is not None and rgba_image.size > 1:
        
        
        if i > 60:
            plt.imshow(rgba_image[:, :, :3])  
            plt.axis("off")
            plt.show()
            i = 0
        else:
            pass
        i += 1

simulation_app.close()
