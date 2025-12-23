from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# 필수 모듈
import carb
import omni
import omni.replicator.core as rep
from pxr import Gf
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.extensions import enable_extension

# 1. Debug Draw 활성화
enable_extension("isaacsim.util.debug_draw")
simulation_app.update()

# 2. 환경(stage) 로드
assets_root_path = get_assets_root_path()
open_stage(assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd")
simulation_app.update()

# 3. World 초기화
from omni.isaac.core import World
my_world = World(stage_units_in_meters=1.0)
#my_world.scene.add_default_ground_plane()

# 4. 큐브 생성
from omni.isaac.core.objects import DynamicCuboid
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np

cube = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube",
        name="cube",
        position=np.array([0.0, -3.0, 1.0]),
        orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
        scale=np.array([0.5, 0.5, 0.5]),
        size=1.0,
        color=np.array([0, 0, 255]),
    )
)
my_world.reset()


# 5. Radar 센서 생성
radar_config = "Example"
_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxRadar",
    path="/sensor",
    parent=None,
    config=radar_config,
    translation=(0.0, 0.0, 0.25),
    orientation=Gf.Quatd(1.0, 0, 0, 0.0),
)

# 6. Render Product 생성 (해상도 최소 512x512 이상)
render_product = rep.create.render_product(sensor.GetPath(), resolution=(512, 512))


# Annotator 등록
annotator = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
annotator.attach(render_product)

# 7. Radar Writer 등록
writer = rep.writers.get("RtxRadarDebugDrawPointCloud")
writer.attach([render_product])

# 8. SimulationContext 초기화
simulation_context = SimulationContext(physics_dt=1.0/60.0, rendering_dt=1.0/60.0, stage_units_in_meters=1.0)
simulation_app.update()
simulation_context.play()

# 9. 시뮬레이션 루프
while simulation_app.is_running(): 
    simulation_app.update()
    data = annotator.get_data()
    print(data['data'])

# 종료
simulation_app.close()

