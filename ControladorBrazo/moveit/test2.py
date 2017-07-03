import rospy
import actionlib
from tf.listener import TransformListener
from geometry_msgs.msg import *
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive


## @brief Pure python interface to move_group action
class MoveGroupInterface(object):

    ## @brief Constructor for this utility
    ## @param group Name of the MoveIt! group to command
    ## @param frame Name of the fixed frame in which planning happens
    ## @param listener A TF listener instance (optional, will create a new one if None)
    ## @param plan_only Should we only plan, but not execute?
    def __init__(self, group, frame, listener=None, plan_only=False):
        self._group = group
        self._fixed_frame = frame
        self._action = actionlib.SimpleActionClient('move_group',
                                                    MoveGroupAction)
        self._action.wait_for_server()
        if listener == None:
            self._listener = TransformListener()
        else:
            self._listener = listener
        self.plan_only = plan_only
        self.planner_id = None
        self.planning_time = 15.0

    def get_move_action(self):
        return self._action

    ## @brief Move the arm to set of joint position goals
    def moveToJointPosition(self,
                            joints,
                            positions,
                            tolerance=0.01,
                            wait=True,
                            **kwargs):
        # Check arguments
        supported_args = ("max_velocity_scaling_factor",
                          "planner_id",
                          "planning_scene_diff",
                          "planning_time",
                          "plan_only",
                          "start_state")
        for arg in kwargs.keys():
            if not arg in supported_args:
                rospy.loginfo("moveToJointPosition: unsupported argument: %s",
                              arg)

        # Create goal
        g = MoveGroupGoal()

        # 1. fill in workspace_parameters

        # 2. fill in start_state
        try:
            g.request.start_state = kwargs["start_state"]
        except KeyError:
            g.request.start_state.is_diff = True

        # 3. fill in goal_constraints
        c1 = Constraints()
        for i in range(len(joints)):
            c1.joint_constraints.append(JointConstraint())
            c1.joint_constraints[i].joint_name = joints[i]
            c1.joint_constraints[i].position = positions[i]
            c1.joint_constraints[i].tolerance_above = tolerance
            c1.joint_constraints[i].tolerance_below = tolerance
            c1.joint_constraints[i].weight = 1.0
        g.request.goal_constraints.append(c1)

        # 4. fill in path constraints

        # 5. fill in trajectory constraints

        # 6. fill in planner id
        try:
            g.request.planner_id = kwargs["planner_id"]
        except KeyError:
            if self.planner_id:
                g.request.planner_id = self.planner_id

        # 7. fill in group name
        g.request.group_name = self._group

        # 8. fill in number of planning attempts
        try:
            g.request.num_planning_attempts = kwargs["num_attempts"]
        except KeyError:
            g.request.num_planning_attempts = 1

        # 9. fill in allowed planning time
        try:
            g.request.allowed_planning_time = kwargs["planning_time"]
        except KeyError:
            g.request.allowed_planning_time = self.planning_time

        # Fill in velocity scaling factor
        try:
            g.request.max_velocity_scaling_factor = kwargs["max_velocity_scaling_factor"]
        except KeyError:
            pass  # do not fill in at all

        # 10. fill in planning options diff
        try:
            g.planning_options.planning_scene_diff = kwargs["planning_scene_diff"]
        except KeyError:
            g.planning_options.planning_scene_diff.is_diff = True
            g.planning_options.planning_scene_diff.robot_state.is_diff = True

        # 11. fill in planning options plan only
        try:
            g.planning_options.plan_only = kwargs["plan_only"]
        except KeyError:
            g.planning_options.plan_only = self.plan_only

        # 12. fill in other planning options
        g.planning_options.look_around = False
        g.planning_options.replan = False

        # 13. send goal
        print("-----------este es el goal que manda, a ver que tiene\n")
        print(g)
        print("\n")
        self._action.send_goal(g)
        print("-----------aqui manda el goal\n")
        if wait:
            self._action.wait_for_result()
            print("********************* se supone que ya ha acabado y aqui devuelve cosas\n")
            return self._action.get_result()
        else:
            return None


    ## @brief Sets the planner_id used for all future planning requests.
    ## @param planner_id The string for the planner id, set to None to clear
    def setPlannerId(self, planner_id):
        self.planner_id = str(planner_id)

    ## @brief Set default planning time to be used for future planning request.
    def setPlanningTime(self, time):
        self.planning_time = time



if __name__=='__main__':
  try:
    print("Hola, estoy probando el test 2. Voy a arrancar un nodo y crear el objeto")
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    test = MoveGroupInterface("gear", "/world")
    print("me he creado el objeto y no ha petado ")
    joint = [
		'elbow_joint',
		'linear_arm_actuator_joint',
		'shoulder_lift_joint',
		'shoulder_pan_joint',
		'wrist_1_joint',
		'wrist_2_joint',
		'wrist_3_joint',
		]
    position = [1.5378, 1.5378, 1.5378, 1.5378, 1.5378, 1.5378, 1.5378]
    print("he definido las constantes posicion y joint y voy a definir estados del planificador a ver si cuela")
    test.setPlannerId("OMPL")
    print("voy a llamar a mover")
    print(test.moveToJointPosition(joint, position))
    print("lo he llamado y no ha petado nada yujuuuu")
  except rospy.ROSInterruptException:
    pass



