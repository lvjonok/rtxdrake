import os
import pathlib
from dataclasses import dataclass

import numpy.typing as npt
from drake import lcmt_viewer_geometry_data
from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
from pxr import Gf, Sdf

# Define a temporary directory to store the USDA files.
# Uses the environment variable 'TEMP_USDA_DIR' if set; otherwise defaults to '/tmp/tmp_usda'.
TEMP_USDA_DIR = os.environ.get("TEMP_USDA_DIR", "/tmp/tmp_usda")


def convert_from_gltf_to_usda(gltf_path: str, usda_path: str):
    """
    Convert a .gltf file to a .usda file using the 'guc' command-line tool.

    Args:
        gltf_path (str): The path to the input .gltf file.
        usda_path (str): The path to the output .usda file.
    """
    # Execute the conversion command. Ensure that 'guc' is installed and in the system's PATH.
    os.system(f"guc {gltf_path} {usda_path}")


@dataclass
class Geom:
    """
    A data class representing geometric data for visualization in a USD stage.

    Attributes:
        name (str): The name identifier for the geometry.
        position (npt.ArrayLike): The position of the geometry in 3D space.
        quaternion (npt.ArrayLike): The orientation of the geometry as a quaternion.
        color (npt.ArrayLike): The color of the geometry.
        string_data (str): Path to the USDA file generated from a GLTF file.
        stage_path (str): The USD stage path where the geometry will be placed.
        should_add (bool): Flag indicating whether to add the geometry to the stage.
    """

    name: str
    position: npt.ArrayLike
    quaternion: npt.ArrayLike
    color: npt.ArrayLike
    string_data: str  # Currently holds the path to a USDA file generated from GLTF.

    # USD stage path to access the prim element.
    stage_path: str

    # Flag to determine if the geometry should be added to the USD stage.
    should_add: bool = True

    @staticmethod
    def from_geometry_data(
        msg: lcmt_viewer_geometry_data,
        root: str = "/World/",
        name: str = "",
    ) -> "Geom":
        """
        Create a Geom instance from geometry data received from Drake's visualization.

        Args:
            msg (lcmt_viewer_geometry_data): The geometry data message from Drake.
            root (str, optional): The root path in the USD stage. Defaults to "/World/".
            name (str, optional): The name identifier for the geometry. Defaults to "".

        Returns:
            Geom: An instance of the Geom class populated with the provided data.
        """
        # Initialize variables for USDA path and addition flag.
        usda_path = pathlib.Path("")
        should_add = False

        # Process geometry based on its type.
        match msg.type:
            case msg.MESH:
                # Handle mesh geometry; currently assumes GLTF format.
                mesh_path = msg.string_data

                # Validate that the mesh path ends with '.gltf'.
                if not mesh_path.endswith(".gltf"):
                    print("ERROR: Mesh path does not end with .gltf")
                    # Optionally, raise an exception to halt execution.
                    # raise ValueError("Mesh path does not end with .gltf")

                # Convert the GLTF file to USDA format and store it in the temporary directory.
                geom_name = name
                usda_path = pathlib.Path(TEMP_USDA_DIR) / f"{geom_name}.usda"
                convert_from_gltf_to_usda(mesh_path, usda_path)
                should_add = True

        # Create and return the Geom instance with the processed data.
        return Geom(
            name=name,
            position=msg.position,
            quaternion=msg.quaternion,
            color=msg.color,
            string_data=str(usda_path.absolute()),
            stage_path=f"{root}{name}",
            should_add=should_add,
        )

    def add_to_stage(self):
        """
        Add the geometry to the USD stage if the 'should_add' flag is True.

        This method creates a USD prim, sets its transformation attributes,
        and adds a payload referencing the USDA file.
        """
        if not self.should_add:
            return

        print(f"Adding {self.name} to the stage")

        # Create a new prim at the specified stage path.
        xform = create_prim(self.stage_path, "")

        # Add the USDA file as a payload to the prim.
        # Assumes that 'string_data' contains a valid USDA file path.
        xform.GetPayloads().AddPayload(str(self.string_data))

        # Set the translation (position) of the prim.
        xform_translate_attr = xform.GetAttribute("xformOp:translate")
        if not xform_translate_attr:
            xform_translate_attr = xform.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Float3)
        xform_translate_attr.Set(Gf.Vec3f(*self.position))

        # Set the scale of the prim to a uniform unit scale.
        xform_scale_attr = xform.GetAttribute("xformOp:scale")
        if not xform_scale_attr:
            xform_scale_attr = xform.CreateAttribute("xformOp:scale", Sdf.ValueTypeNames.Float3)
        xform_scale_attr.Set(Gf.Vec3f(1.0, 1.0, 1.0))

        # Set the orientation (rotation) of the prim using the quaternion.
        xform_orient_attr = xform.GetAttribute("xformOp:orient")
        if not xform_orient_attr:
            xform_orient_attr = xform.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatf)
        xform_orient_attr.Set(
            Gf.Quatd(
                self.quaternion[0],
                Gf.Vec3d(self.quaternion[1], self.quaternion[2], self.quaternion[3]),
            )
        )

        # Set the rotation unit resolution for the X-axis rotation.
        xform_rotate_x_units_resolve_attr = xform.GetAttribute("xformOp:rotateX:unitsResolve")
        if not xform_rotate_x_units_resolve_attr:
            xform_rotate_x_units_resolve_attr = xform.CreateAttribute(
                "xformOp:rotateX:unitsResolve", Sdf.ValueTypeNames.Double
            )
        xform_rotate_x_units_resolve_attr.Set(90)

        # Define the order of transformation operations to ensure consistent behavior.
        xform_op_order_attr = xform.GetAttribute("xformOpOrder")
        if not xform_op_order_attr:
            xform_op_order_attr = xform.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray)
        xform_op_order_attr.Set(
            [
                "xformOp:translate",
                "xformOp:orient",
                "xformOp:scale",
                "xformOp:rotateX:unitsResolve",
            ]
        )

    def update_draw(
        self,
        position: npt.ArrayLike,
        quaternion: npt.ArrayLike,
    ):
        """
        Update the geometry's position and orientation in the USD stage.

        Args:
            position (npt.ArrayLike): The new position for the geometry.
            quaternion (npt.ArrayLike): The new orientation as a quaternion.
        """
        if not self.should_add:
            return

        # Retrieve the existing prim from the USD stage.
        xform = get_prim_at_path(self.stage_path)
        if not xform:
            print(f"ERROR: Could not find prim {self.stage_path}")
            return

        # Update the translation (position) attribute.
        xform_translate_attr = xform.GetAttribute("xformOp:translate")
        if xform_translate_attr:
            xform_translate_attr.Set(Gf.Vec3f(*position))
        else:
            print(f"WARNING: 'xformOp:translate' attribute not found for prim {self.stage_path}")

        # Update the orientation (quaternion) attribute.
        xform_orient_attr = xform.GetAttribute("xformOp:orient")
        if xform_orient_attr:
            xform_orient_attr.Set(
                Gf.Quatd(
                    quaternion[0],
                    Gf.Vec3d(quaternion[1], quaternion[2], quaternion[3]),
                )
            )
        else:
            print(f"WARNING: 'xformOp:orient' attribute not found for prim {self.stage_path}")
