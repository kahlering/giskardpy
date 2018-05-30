from collections import OrderedDict, defaultdict
from copy import copy
import rospy
from giskard_msgs.msg import Controller

from giskardpy.input_system import JointStatesInput, FrameInput, Point3Input, Vector3Input, ShortestAngularDistanceInput
from giskardpy.plugin import Plugin
from giskardpy.symengine_controller import SymEngineController, position_conv, rotation_conv, \
    link_to_link_avoidance, joint_position, continuous_joint_position, link_to_link_avoidance_old
import symengine_wrappers as sw
from giskardpy.utils import keydefaultdict


class CartesianBulletControllerPlugin(Plugin):
    def __init__(self, root_link, js_identifier, fk_identifier, goal_identifier, next_cmd_identifier,
                 collision_identifier, closest_point_identifier, controlled_joints_identifier,
                 collision_goal_identifier, path_to_functions):
        """
        :param roots:
        :type roots: list
        :param tips:
        :type tips: list
        :param js_identifier:
        :param fk_identifier:
        :param goal_identifier:
        :param next_cmd_identifier:
        :param collision_identifier:
        :param closest_point_identifier:
        """
        self.collision_goal_identifier = collision_goal_identifier
        self.controlled_joints_identifier = controlled_joints_identifier
        self._pyfunctions_identifier = 'pyfunctions'
        self._fk_identifier = fk_identifier
        self._collision_identifier = collision_identifier
        self._closest_point_identifier = closest_point_identifier
        self.path_to_functions = path_to_functions
        self.root = root_link
        self.soft_constraints = OrderedDict()
        self._joint_states_identifier = js_identifier
        self._goal_identifier = goal_identifier
        self._next_cmd_identifier = next_cmd_identifier
        self._controller = None
        self.known_constraints = set()
        self.controlled_joints = set()
        self.controller_updated = False
        self.max_number_collision_entries = 10
        super(CartesianBulletControllerPlugin, self).__init__()

    # @profile
    def update(self):
        # update controlled joints
        new_controlled_joints = self.god_map.get_data([self.controlled_joints_identifier])
        if len(set(new_controlled_joints).difference(self.controlled_joints)) != 0:
            self._controller.set_controlled_joints(new_controlled_joints)
        self.controlled_joints = new_controlled_joints
        self.init_controller()

        if self.god_map.get_data([self._goal_identifier]) is not None:
            # TODO set joint goals

            if not self.controller_updated:
                self.modify_controller()

            expr = self.god_map.get_expr_values()
            next_cmd = self._controller.get_cmd(expr)
            self.next_cmd.update(next_cmd)

        if len(self.next_cmd) > 0:
            self.god_map.set_data([self._next_cmd_identifier], self.next_cmd)

    def start_always(self):
        self.next_cmd = {}
        self.rebuild_controller = False

    def stop(self):
        pass

    def init_controller(self):
        if not self._controller.is_initialized():
            robot = self._controller.robot
            self.rebuild_controller = True
            controllable_links = set()
            pyfunctions = {}
            for joint_name in self.controlled_joints:
                current_joint_key = [self._joint_states_identifier, joint_name, 'position']
                goal_joint_key = [self._goal_identifier, str(Controller.JOINT), joint_name, 'position']
                current_joint_expr = self.god_map.get_expr(current_joint_key)
                goal_joint_expr = self.god_map.get_expr(goal_joint_key)
                weight = self.god_map.get_expr([self._goal_identifier, str(Controller.JOINT), joint_name, 'weight'])
                gain = self.god_map.get_expr([self._goal_identifier, str(Controller.JOINT), joint_name, 'p_gain'])
                max_speed = self.god_map.get_expr(
                    [self._goal_identifier, str(Controller.JOINT), joint_name, 'max_speed'])
                if robot.is_continuous(joint_name):
                    change = ShortestAngularDistanceInput(self.god_map.get_expr,
                                                          [self._pyfunctions_identifier],
                                                          current_joint_key,
                                                          goal_joint_key)
                    pyfunctions[change.get_key()] = change
                    self.soft_constraints[joint_name] = continuous_joint_position(current_joint_expr,
                                                                                  change.get_expression(),
                                                                                  weight,
                                                                                  gain,
                                                                                  max_speed)
                else:
                    self.soft_constraints[joint_name] = joint_position(current_joint_expr, goal_joint_expr, weight,
                                                                       gain,
                                                                       max_speed)
                controllable_links.update(robot.get_link_tree(joint_name))
            self.god_map.set_data([self._pyfunctions_identifier], pyfunctions)

            for link in list(controllable_links):
                point_on_link_input = Point3Input.prefix(self.god_map.get_expr,
                                                         [self._closest_point_identifier, link, 'position_on_a'])
                other_point_input = Point3Input.prefix(self.god_map.get_expr,
                                                       [self._closest_point_identifier, link, 'position_on_b'])
                trans_prefix = '{}/{},{}/pose/position'.format(self._fk_identifier, self.root, link)
                rot_prefix = '{}/{},{}/pose/orientation'.format(self._fk_identifier, self.root, link)
                current_input = FrameInput.prefix_constructor(trans_prefix, rot_prefix, self.god_map.get_expr)
                min_dist = self.god_map.get_expr([self._closest_point_identifier, link, 'min_dist'])
                contact_normal = Vector3Input.prefix(self.god_map.get_expr,
                                                     [self._closest_point_identifier, link, 'contact_normal'])

                # self.soft_constraints.update(link_to_link_avoidance_old(link,
                #                                                         robot.get_fk_expression(self.root, link),
                #                                                         current_input.get_frame(),
                #                                                         point_on_link_input.get_expression(),
                #                                                         other_point_input.get_expression(),
                #                                                         min_dist))
                self.soft_constraints.update(link_to_link_avoidance(link,
                                                                    robot.get_fk_expression(self.root, link),
                                                                    current_input.get_frame(),
                                                                    point_on_link_input.get_expression(),
                                                                    other_point_input.get_expression(),
                                                                    contact_normal.get_expression(),
                                                                    min_dist))

    def start_once(self):
        urdf = rospy.get_param('robot_description')
        self._controller = SymEngineController(urdf, self.path_to_functions)
        robot = self._controller.robot

        current_joints = JointStatesInput.prefix_constructor(self.god_map.get_expr,
                                                             robot.get_joint_names(),
                                                             self._joint_states_identifier,
                                                             'position')
        robot.set_joint_symbol_map(current_joints)

    def modify_controller(self):
        robot = self._controller.robot
        hold_joints = set(self.controlled_joints)

        for (root, tip), value in self.god_map.get_data([self._goal_identifier,
                                                         str(Controller.TRANSLATION_3D)]).items():
            key = '{}/{},{}'.format(Controller.TRANSLATION_3D, root, tip)
            hold_joints.difference_update(robot.get_chain_joints(root, tip))
            if key not in self.known_constraints:
                print('added chain {} -> {} type: TRANSLATION_3D'.format(root, tip))
                self.known_constraints.add(key)
                self.soft_constraints.update(self.controller_msg_to_constraint(root, tip, Controller.TRANSLATION_3D))
                self.rebuild_controller = True
            self.god_map.set_data([self._goal_identifier,
                                   str(Controller.TRANSLATION_3D),
                                   ','.join([root, tip]),
                                   'weight'], 1)
        for (root, tip), value in self.god_map.get_data([self._goal_identifier, str(Controller.ROTATION_3D)]).items():
            key = '{}/{},{}'.format(Controller.ROTATION_3D, root, tip)
            hold_joints.difference_update(robot.get_chain_joints(root, tip))
            if key not in self.known_constraints:
                print('added chain {} -> {} type: ROTATION_3D'.format(root, tip))
                self.known_constraints.add(key)
                self.soft_constraints.update(self.controller_msg_to_constraint(root, tip, Controller.ROTATION_3D))
                self.rebuild_controller = True
            self.god_map.set_data([self._goal_identifier,
                                   str(Controller.ROTATION_3D),
                                   ','.join([root, tip]),
                                   'weight'], 1)

        # TODO handle joint controller

        # set weight of unused joints to 0
        joint_goal = self.god_map.get_data([self._goal_identifier, str(Controller.JOINT)])
        for joint_name in self.controlled_joints:
            if joint_name not in joint_goal:
                joint_goal[joint_name] = {'weight': 0,
                                          'position': self.god_map.get_data([self._joint_states_identifier,
                                                                             joint_name,
                                                                             'position'])}
                if joint_name in hold_joints:
                    joint_goal[joint_name]['weight'] = 1

        self.god_map.set_data([self._goal_identifier, str(Controller.JOINT)], joint_goal)

        if self.rebuild_controller or self._controller is None:
            # TODO sanity checking

            self._controller.init(self.soft_constraints, self.god_map.get_free_symbols())
            self.rebuild_controller = False
        self.controller_updated = True

    def controller_msg_to_constraint(self, root, tip, type):
        """
        :param controller_msg:
        :type controller_msg: Controller
        :return:
        :rtype: dict
        """
        robot = self._controller.robot

        trans_prefix = '{}/{}/{},{}/goal_pose/pose/position'.format(self._goal_identifier, Controller.TRANSLATION_3D,
                                                                    root, tip)
        rot_prefix = '{}/{}/{},{}/goal_pose/pose/orientation'.format(self._goal_identifier, Controller.ROTATION_3D,
                                                                     root, tip)
        goal_input = FrameInput.prefix_constructor(trans_prefix, rot_prefix, self.god_map.get_expr)

        trans_prefix = '{}/{},{}/pose/position'.format(self._fk_identifier, root, tip)
        rot_prefix = '{}/{},{}/pose/orientation'.format(self._fk_identifier, root, tip)
        current_input = FrameInput.prefix_constructor(trans_prefix, rot_prefix, self.god_map.get_expr)
        weight = self.god_map.get_expr([self._goal_identifier, str(type), ','.join([root, tip]), 'weight'])
        p_gain = self.god_map.get_expr([self._goal_identifier, str(type), ','.join([root, tip]), 'p_gain'])
        max_speed = self.god_map.get_expr([self._goal_identifier, str(type), ','.join([root, tip]), 'max_speed'])

        if type == Controller.TRANSLATION_3D:
            return position_conv(goal_input.get_position(),
                                 sw.pos_of(robot.get_fk_expression(root, tip)),
                                 weights=weight,
                                 trans_gain=p_gain,
                                 max_trans_speed=max_speed,
                                 ns='{}/{}'.format(root, tip))
        elif type == Controller.ROTATION_3D:
            return rotation_conv(goal_input.get_rotation(),
                                 sw.rot_of(robot.get_fk_expression(root, tip)),
                                 current_input.get_rotation(),
                                 weights=weight,
                                 rot_gain=p_gain,
                                 max_rot_speed=max_speed,
                                 ns='{}/{}'.format(root, tip))

        return {}

    def copy(self):
        cp = self.__class__(self.root, self._joint_states_identifier, self._fk_identifier,
                            self._goal_identifier, self._next_cmd_identifier, self._collision_identifier,
                            self._closest_point_identifier, self.controlled_joints_identifier,
                            self.collision_goal_identifier, self.path_to_functions)
        # TODO not cool that you always have to copy the controller here
        cp._controller = self._controller
        cp.soft_constraints = self.soft_constraints
        cp.known_constraints = self.known_constraints
        # cp.controlled_joints = self.controlled_joints
        return cp
