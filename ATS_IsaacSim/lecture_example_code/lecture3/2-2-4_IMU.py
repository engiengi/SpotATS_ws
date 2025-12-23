from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()


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

from omni.isaac.sensor import IMUSensor


sensor = my_world.scene.add(IMUSensor(
    prim_path="/World/cube/Imu",
    name="imu",
    frequency=60,
    translation=np.array([0, 0, 0.5]),
    orientation=np.array([1, 0, 0, 0]),
    linear_acceleration_filter_size=10,
    angular_velocity_filter_size=10,
    orientation_filter_size=10,
))
my_world.reset()
import omni.ui as ui

window = ui.Window("IMU HUD", width=300, height=100, dockPreference=ui.DockPreference.RIGHT_TOP)

with window.frame:
    with ui.VStack():
        imu_label = ui.Label("Waiting for IMU data...")

while simulation_app.is_running():
    my_world.step(render=True)
    imu_data = sensor.get_current_frame(read_gravity=True)["lin_acc"]
    imu_label.text = f"IMU Accel: {np.round(imu_data, 3)}"
simulation_app.close()
