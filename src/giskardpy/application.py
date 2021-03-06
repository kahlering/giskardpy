import traceback

import rospy

class ROSApplication(object):
    """Superclass for all giskard ROS applications. Runs a process manager until it terminates."""
    # TODO do we still need this class?
    def __init__(self, process_manager):
        self.process_manager = process_manager

    def run(self):
        try:
            self.process_manager.start_loop()
        except Exception as e:
            traceback.print_exc()
        finally:
            self.process_manager.stop()