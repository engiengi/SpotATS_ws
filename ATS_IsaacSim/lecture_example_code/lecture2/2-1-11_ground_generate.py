from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
my_world.reset()

while simulation_app.is_running():
    my_world.step(render=True)

simulation_app.close()
