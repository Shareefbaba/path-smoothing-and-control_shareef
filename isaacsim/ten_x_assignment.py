import sys

from isaacsim import SimulationApp
ROBOT_USD = "/home/shareef/ros2_ws/src/tenx_assignment/isaacsim/usd/10x_project.usd"
CONFIG = {"renderer": "RaytracedLighting", "headless": False}

simulation_app = SimulationApp(CONFIG)

import numpy as np
from isaacsim.core.utils import extensions, stage
from isaacsim.core.api import SimulationContext
from isaacsim.storage.native import get_assets_root_path
import carb

extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
Simulation_context = SimulationContext(stage_units_in_meters=1.0)

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find assets root path. Is the Isaac Sim installation corrupted?")
    simulation_app.close()
    sys.exit()

# Load your USD under /World
stage.add_reference_to_stage(ROBOT_USD, "/World")
simulation_app.update()


# Keep the app open: run the sim loop until you close the window
Simulation_context.initialize_physics()
Simulation_context.play()

frame = 0
while simulation_app.is_running():
    Simulation_context.step(render=True)

Simulation_context.stop()
simulation_app.close()
