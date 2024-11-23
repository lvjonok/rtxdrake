import lcm
from drake import lcmt_viewer_draw, lcmt_viewer_load_robot


class Listener:
    """
    Listener class to handle incoming LCM messages related to robot visualization.

    This class subscribes to specific LCM channels and invokes designated handlers
    when new messages are received. It facilitates communication between Drake's
    visualization tools and your application by processing load and draw messages.
    """

    def __init__(self, load_handler: callable = None, draw_handler: callable = None):
        """
        Initialize the Listener with optional handlers for load and draw messages.

        Args:
            load_handler (callable, optional): Function to handle 'DRAKE_VIEWER_LOAD_ROBOT' messages.
                Should accept a single argument of type lcmt_viewer_load_robot.
            draw_handler (callable, optional): Function to handle 'DRAKE_VIEWER_DRAW' messages.
                Should accept a single argument of type lcmt_viewer_draw.
        """
        # Store the provided handler functions for later use.
        self.__load_handler = load_handler
        self.__draw_handler = draw_handler

        # Initialize the LCM instance for communication.
        self.lc: lcm.LCM = lcm.LCM()

        # Subscribe to the 'DRAKE_VIEWER_LOAD_ROBOT' channel with the load_handler.
        # If no load_handler is provided, a default handler that does nothing is used.
        self.load_sub = self.lc.subscribe(
            "DRAKE_VIEWER_LOAD_ROBOT", self.load_handler if self.__load_handler else self.default_load_handler
        )

        # Subscribe to the 'DRAKE_VIEWER_DRAW' channel with the draw_handler.
        # If no draw_handler is provided, a default handler that does nothing is used.
        self.draw_sub = self.lc.subscribe(
            "DRAKE_VIEWER_DRAW", self.draw_handler if self.__draw_handler else self.default_draw_handler
        )

    def load_handler(self, channel: str, data: bytes):
        """
        Internal handler for 'DRAKE_VIEWER_LOAD_ROBOT' messages.

        Decodes the incoming data and invokes the user-provided load_handler.

        Args:
            channel (str): The name of the channel from which the message was received.
            data (bytes): The raw message data to be decoded.
        """
        try:
            # Decode the incoming data into an lcmt_viewer_load_robot message.
            decoded_data = lcmt_viewer_load_robot.decode(data)
            if self.__load_handler:
                # Invoke the user-provided handler with the decoded data.
                self.__load_handler(decoded_data)
            else:
                # If no handler is provided, use the default handler.
                self.default_load_handler(decoded_data)
        except Exception as e:
            # Handle any exceptions that occur during decoding or handling.
            print(f"Error in load_handler: {e}")

    def draw_handler(self, channel: str, data: bytes):
        """
        Internal handler for 'DRAKE_VIEWER_DRAW' messages.

        Decodes the incoming data and invokes the user-provided draw_handler.

        Args:
            channel (str): The name of the channel from which the message was received.
            data (bytes): The raw message data to be decoded.
        """
        try:
            # Decode the incoming data into an lcmt_viewer_draw message.
            decoded_data = lcmt_viewer_draw.decode(data)
            if self.__draw_handler:
                # Invoke the user-provided handler with the decoded data.
                self.__draw_handler(decoded_data)
            else:
                # If no handler is provided, use the default handler.
                self.default_draw_handler(decoded_data)
        except Exception as e:
            # Handle any exceptions that occur during decoding or handling.
            print(f"Error in draw_handler: {e}")

    def default_load_handler(self, msg: lcmt_viewer_load_robot):
        """
        Default handler for 'DRAKE_VIEWER_LOAD_ROBOT' messages.

        This method is called if no user-provided load_handler is supplied.
        It currently performs no action but can be overridden as needed.

        Args:
            msg (lcmt_viewer_load_robot): The decoded load robot message.
        """
        print("Received 'DRAKE_VIEWER_LOAD_ROBOT' message, but no handler is set.")

    def default_draw_handler(self, msg: lcmt_viewer_draw):
        """
        Default handler for 'DRAKE_VIEWER_DRAW' messages.

        This method is called if no user-provided draw_handler is supplied.
        It currently performs no action but can be overridden as needed.

        Args:
            msg (lcmt_viewer_draw): The decoded draw message.
        """
        print("Received 'DRAKE_VIEWER_DRAW' message, but no handler is set.")

    def handle(self):
        """
        Handle incoming LCM messages.

        This method should be called regularly (e.g., within a loop) to process
        incoming messages and invoke the appropriate handlers.
        """
        try:
            # Process a single incoming message, blocking until one is received.
            self.lc.handle()
        except KeyboardInterrupt:
            # Allow graceful shutdown on keyboard interrupt (Ctrl+C).
            print("Listener interrupted by user.")
        except Exception as e:
            # Handle any other exceptions that occur during message handling.
            print(f"Error while handling messages: {e}")
