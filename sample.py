"""Modified version of example translation of `cart_pole_passive_simluation.cc`."""

import argparse
import random

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target_realtime_rate",
        type=float,
        default=1.0,
        help="Desired rate relative to real time. See documentation for "
        "Simulator::set_target_realtime_rate() for details.",
    )
    parser.add_argument(
        "--simulation_time",
        type=float,
        default=100.0,
        help="Desired duration of the simulation in seconds.",
    )
    parser.add_argument(
        "--time_step",
        type=float,
        default=0.0,
        help="If greater than zero, the plant is modeled as a system with "
        "discrete updates and period equal to this time_step. "
        "If 0, the plant is modeled as a continuous system.",
    )
    args = parser.parse_args()

    # Build the diagram
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder=builder, time_step=args.time_step)

    # Load the iiwa14 model
    Parser(plant=plant).AddModelsFromUrl(url="package://drake_models/iiwa_description/urdf/iiwa14_no_collision.urdf")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    plant.Finalize()
    AddDefaultVisualization(builder=builder)
    diagram = builder.Build()

    # Create the simulation context
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(diagram_context)

    # Get joints by name and apply random movements
    joint_names = [f"iiwa_joint_{i}" for i in range(1, 8)]  # iiwa14 has 7 joints
    for joint_name in joint_names:
        joint = plant.GetJointByName(joint_name)
        random_angle = random.uniform(-1.57, 1.57)  # Random angle between -90 and +90 degrees
        joint.set_angle(context=plant_context, angle=random_angle)

    # Set up and run the simulator
    simulator = Simulator(diagram, diagram_context)
    simulator.set_publish_every_time_step(True)
    simulator.set_target_realtime_rate(args.target_realtime_rate)
    simulator.Initialize()
    simulator.AdvanceTo(args.simulation_time)


if __name__ == "__main__":
    main()
