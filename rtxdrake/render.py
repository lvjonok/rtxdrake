import select

from drake import lcmt_viewer_draw, lcmt_viewer_load_robot

from .link import Link
from .listener import Listener


class RenderManager:
    links: list[Link]

    def __init__(self):
        self.links = []
        self.listener = Listener(self.load_handler, self.draw_handler)
        self.timeout = 1e-4  # should be fast enough so we cannot feel delay in isaac sim window

    def load_handler(self, msg: lcmt_viewer_load_robot):
        for lidx in range(msg.num_links):
            link_data = msg.link[lidx]
            link = Link.from_link_data(link_data, name=f"{lidx}")
            self.links.append(link)

            link.add_to_stage()

    def draw_handler(self, msg: lcmt_viewer_draw):
        for idx in range(msg.num_links):
            link: Link = self.links[idx]
            link.update_draw(msg.position[idx], msg.quaternion[idx])

    def fast_handle(self):
        # handle, but with very low timeout
        rfds, _, _ = select.select([self.listener.lc.fileno()], [], [], self.timeout)
        if rfds:
            self.listener.handle()


manager = RenderManager()


def do_work():
    manager.fast_handle()
