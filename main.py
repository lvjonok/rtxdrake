import torch  # WTF??? it didn't link the other way, should import beforehand  # noqa: F401
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False, "width": 1920, "height": 1080})

import omni.isaac.core.utils.stage as stage_utils  # noqa: E402
from omni.isaac.core import World  # noqa: E402

from rtxdrake import do_work  # noqa: E402

if __name__ == "__main__":
    simulation_world = World(stage_units_in_meters=1.0)
    # load the scene
    stage_utils.open_stage("assets/myscene.usda")

    # wait for the user to close the window
    while simulation_app.is_running():
        do_work()

        simulation_app.update()
