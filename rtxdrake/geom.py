import os
import pathlib
from dataclasses import dataclass

import numpy.typing as npt
from drake import lcmt_viewer_geometry_data
from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
from pxr import Gf, Sdf

# define a temporary directory to store the usda files
TEMP_USDA_DIR = "/tmp/tmp_usda" if os.environ.get("TEMP_USDA_DIR") is None else os.environ.get("TEMP_USDA_DIR")


def convert_from_gltf_to_usda(gltf_path: str, usda_path: str):
    """
    Convert a .gltf file to a .usda file.

    Args:
        gltf_path (str): The path to the input .gltf file.
        usda_path (str): The path to the output .usda file.
    """
    os.system(f"guc {gltf_path} {usda_path}")


@dataclass
class Geom:
    name: str
    position: npt.ArrayLike
    quaternion: npt.ArrayLike
    color: npt.ArrayLike
    string_data: str  # right now it is a path to a usda generated from gltf

    # path used to access the stage element
    stage_path: str

    # whether we should add it
    should_add: bool = True

    @staticmethod
    def from_geometry_data(
        msg: lcmt_viewer_geometry_data,
        root: str = "/World/",
        name: str = "",
    ) -> "Geom":
        # we should process this geom somehow
        usda_path = pathlib.Path("")
        should_add = False
        match msg.type:
            case msg.MESH:
                # print("Parse mesh")
                # for now we assume .gltf file
                mesh_path = msg.string_data
                if not mesh_path.endswith(".gltf"):
                    print("ERROR: Mesh path does not end with .gltf")
                    # raise ValueError("Mesh path does not end with .gltf")

                # we want to convert it into .usda as save nearby
                geom_name = name
                usda_path = pathlib.Path(TEMP_USDA_DIR) / f"{geom_name}.usda"
                # print(f"Converting {mesh_path} to {usda_path}")
                convert_from_gltf_to_usda(mesh_path, usda_path)
                should_add = True

        return Geom(
            name=name,
            position=msg.position,
            quaternion=msg.quaternion,
            color=msg.color,
            string_data=usda_path.absolute(),
            stage_path=f"{root}{name}",
            should_add=should_add,
        )

    def add_to_stage(self):
        if not self.should_add:
            return

        print(f"Adding {self.name} to the stage")
        xform = create_prim(self.stage_path, "")

        # TODO: works only for .gltf converted to .usda
        xform.GetPayloads().AddPayload(str(self.string_data))

        # Set translation (position)
        xform_translate_attr = xform.GetAttribute("xformOp:translate")
        if not xform_translate_attr:
            xform_translate_attr = xform.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Float3)
        xform_translate_attr.Set(Gf.Vec3f(*self.position))

        # Set unit scale
        xform_scale_attr = xform.GetAttribute("xformOp:scale")
        if not xform_scale_attr:
            xform_scale_attr = xform.CreateAttribute("xformOp:scale", Sdf.ValueTypeNames.Float3)
        xform_scale_attr.Set(Gf.Vec3f(1.0, 1.0, 1.0))

        # Set orientation (quaternion)
        xform_orient_attr = xform.GetAttribute("xformOp:orient")
        if not xform_orient_attr:
            xform_orient_attr = xform.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatf)
        xform_orient_attr.Set(
            Gf.Quatd(
                self.quaternion[0],
                Gf.Vec3d(self.quaternion[1], self.quaternion[2], self.quaternion[3]),
            )
        )

        # set "xformOp:rotateX:unitsResolve"
        xform_rotate_x_units_resolve_attr = xform.GetAttribute("xformOp:rotateX:unitsResolve")
        if not xform_rotate_x_units_resolve_attr:
            xform_rotate_x_units_resolve_attr = xform.CreateAttribute(
                "xformOp:rotateX:unitsResolve", Sdf.ValueTypeNames.Double
            )
        xform_rotate_x_units_resolve_attr.Set(90)

        # Set xFormOpOrder to
        # uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
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
        if not self.should_add:
            return

        # xform = stage.GetPrimAtPath(self.stage_path)
        xform = get_prim_at_path(self.stage_path)
        if not xform:
            print(f"ERROR: Could not find prim {self.stage_path}")
            return

        # Set translation (position)
        xform_translate_attr = xform.GetAttribute("xformOp:translate")
        xform_translate_attr.Set(Gf.Vec3f(*position))

        # Set orientation (quaternion)
        xform_orient_attr = xform.GetAttribute("xformOp:orient")
        xform_orient_attr.Set(
            Gf.Quatd(
                quaternion[0],
                Gf.Vec3d(quaternion[1], quaternion[2], quaternion[3]),
            )
        )
