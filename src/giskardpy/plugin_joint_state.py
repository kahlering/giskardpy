from Queue import Empty, Queue

import rospy
from py_trees import Status

from sensor_msgs.msg import JointState

from giskardpy.identifier import js_identifier
from giskardpy.plugin import PluginBase
from giskardpy.utils import to_joint_state_dict


class JointStatePlugin(PluginBase):
    """
    Listens to a joint state topic, transforms it into a dict and writes it to the got map.
    Gets replace with a kinematic sim plugin during a parallel universe.
    """

    def __init__(self):
        """
        :type js_identifier: str
        """
        super(JointStatePlugin, self).__init__()
        self.mjs = None
        self.lock = Queue(maxsize=1)

    def cb(self, data):
        try:
            self.lock.get_nowait()
        except Empty:
            pass
        self.lock.put(data)

    def update(self):
        try:
            if self.mjs is None:
                js = self.lock.get()
            else:
                js = self.lock.get_nowait()
            self.mjs = to_joint_state_dict(js)
        except Empty:
            pass
        self.god_map.safe_set_data([js_identifier], self.mjs)
        return None

    def setup(self):
        self.joint_state_sub = rospy.Subscriber(u'/hsrb/joint_states', JointState, self.cb, queue_size=1)
