from dataclasses import dataclass

from drake import lcmt_viewer_link_data

from rtxdrake import Geom


@dataclass
class Link:
    name: str
    robot_num: int
    num_geom: int
    geoms: list[Geom] = None

    def add_geom(self, geom: Geom):
        if self.geoms is None:
            self.geoms = []
        self.geoms.append(geom)

    @staticmethod
    def from_link_data(
        msg: lcmt_viewer_link_data,
        root: str = "/World/",
        name: str = "",
    ) -> "Link":
        link = Link(name=name, robot_num=msg.robot_num, num_geom=msg.num_geom)
        for geom_idx, geom in enumerate(msg.geom):
            link.add_geom(
                Geom.from_geometry_data(
                    geom,
                    root=root,
                    name=f"link_{link.name}_geom_idx_{geom_idx}",
                )
            )
        return link

    def add_to_stage(self):
        for geom in self.geoms:
            geom.add_to_stage()

    def update_draw(self, position, quaternion):
        for idx in range(self.num_geom):
            self.geoms[idx].update_draw(position, quaternion)
