import lcm
from drake import lcmt_viewer_draw, lcmt_viewer_load_robot


class Listener:
    # this class is responsible to signal about new messages received about the visualization
    def __init__(self, load_handler: callable = None, draw_handler: callable = None):
        self.__load_handler = load_handler
        self.__draw_handler = draw_handler

        self.lc: lcm.LCM = lcm.LCM()
        self.load_sub = self.lc.subscribe("DRAKE_VIEWER_LOAD_ROBOT", self.load_handler)
        self.draw_sub = self.lc.subscribe("DRAKE_VIEWER_DRAW", self.draw_handler)

    def load_handler(self, channel, data):
        data = lcmt_viewer_load_robot.decode(data)
        self.__load_handler(data)

    def draw_handler(self, channel, data):
        data = lcmt_viewer_draw.decode(data)
        self.__draw_handler(data)

    def handle(self):
        self.lc.handle()
