import select
from typing import List

from drake import lcmt_viewer_draw, lcmt_viewer_load_robot

from .link import Link
from .listener import Listener


class RenderManager:
    """
    RenderManager class to manage the rendering of robotic links and their geometries
    based on incoming LCM messages from Drake's visualization tools.

    This class handles the loading of robot models and updating their positions and
    orientations in the USD stage by interacting with Link and Listener classes.
    """

    def __init__(self):
        """
        Initialize the RenderManager.

        Sets up the Listener with appropriate handlers for load and draw messages
        and initializes the list of Link instances. Configures a timeout for message
        handling to ensure responsive updates in the Isaac Sim window.
        """
        # Initialize an empty list to store Link instances.
        self.links: List[Link] = []

        # Instantiate the Listener with the load and draw handlers.
        self.listener = Listener(self.load_handler, self.draw_handler)

        # Set a low timeout to ensure rapid message handling, minimizing perceived delays.
        self.timeout = 1e-4  # 0.0001 seconds

    def load_handler(self, msg: lcmt_viewer_load_robot):
        """
        Handle 'DRAKE_VIEWER_LOAD_ROBOT' messages to load robot links into the USD stage.

        Args:
            msg (lcmt_viewer_load_robot): The message containing data about the robot's links.
        """
        # Iterate over each link in the incoming message based on the number of links.
        for lidx in range(msg.num_links):
            # Extract the data for the current link.
            link_data = msg.link[lidx]

            # Create a Link instance from the extracted link data.
            # The name is assigned based on the link index for uniqueness.
            link = Link.from_link_data(link_data, name=f"{lidx}")

            # Append the newly created Link to the manager's list of links.
            self.links.append(link)

            # Add the Link's geometries to the USD stage for visualization.
            link.add_to_stage()

    def draw_handler(self, msg: lcmt_viewer_draw):
        """
        Handle 'DRAKE_VIEWER_DRAW' messages to update the positions and orientations
        of existing robot links in the USD stage.

        Args:
            msg (lcmt_viewer_draw): The message containing draw commands with positions
                                     and orientations for the links.
        """
        # Iterate over each link's update command in the incoming message.
        for idx in range(msg.num_links):
            try:
                # Retrieve the corresponding Link instance based on the index.
                link: Link = self.links[idx]

                # Update the Link's position and orientation using the provided data.
                link.update_draw(msg.position[idx], msg.quaternion[idx])
            except IndexError:
                # Handle cases where the message contains more links than currently loaded.
                print(f"Warning: Received draw command for non-existent Link index {idx}.")

    def fast_handle(self):
        """
        Handle incoming LCM messages with a very low timeout to ensure rapid processing.

        This method uses the `select` module to wait for incoming messages on the LCM
        file descriptor with a specified timeout. If a message is available, it processes
        the message by invoking the Listener's handle method.
        """
        # Use select to monitor the LCM file descriptor for incoming messages.
        rfds, _, _ = select.select([self.listener.lc.fileno()], [], [], self.timeout)

        # If there are readable file descriptors, handle the incoming message.
        if rfds:
            self.listener.handle()


# Instantiate the RenderManager.
manager = RenderManager()


def do_work():
    """
    Function to perform periodic work by handling incoming LCM messages.

    This function should be called regularly (e.g., within a loop or a scheduled
    callback) to ensure that incoming messages are processed in a timely manner,
    keeping the USD stage up-to-date with the latest robot states.
    """
    manager.fast_handle()
