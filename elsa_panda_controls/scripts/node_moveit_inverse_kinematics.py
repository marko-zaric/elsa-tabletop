import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from elsa_panda_controls.srv import GoalPoseJointSpace, GoalPoseJointSpaceResponse
from franka_core_msgs.msg import EndPointState
import numpy as np
import quaternion

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
CARTESIAN_POSE = None

class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterface"""

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        # print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        # print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def get_final_joints(self, x, y, z, o_x, o_y, o_z, o_w):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        if o_x == 1000.0 and o_y == 1000.0 and o_z == 1000.0 and o_w == 1000.0:
            curr_pose = copy.deepcopy(CARTESIAN_POSE)
            curr_pos, curr_ori = curr_pose['position'], quaternion.as_float_array(curr_pose['orientation'])

            pose_goal.position.x = x
            pose_goal.position.y = y
            pose_goal.position.z = z
            
            pose_goal.orientation.w = curr_ori[0]
            pose_goal.orientation.x = curr_ori[1]
            pose_goal.orientation.y = curr_ori[2]
            pose_goal.orientation.z = curr_ori[3]
            

        else:
            pose_goal.orientation.x = o_x
            pose_goal.orientation.y = o_y
            pose_goal.orientation.z = o_z
            pose_goal.orientation.w = o_w
            pose_goal.position.x = x
            pose_goal.position.y = y
            pose_goal.position.z = z
        
        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        full_plan = move_group.plan()
        success = full_plan[0]
        rospy.loginfo("Inverse Kinematics calculation sucessful: " + str(success))
        if success != True:
            move_group.clear_pose_targets()
            return success, []
        else:
            final_joints = full_plan[-3].joint_trajectory.points[-1].positions
            move_group.clear_pose_targets()
            return success, final_joints



def callback(request):
    rate = rospy.Rate(100)
    try: 
        interface = MoveGroupPythonInterface()
        success, position_joint_space = interface.get_final_joints(request.x, request.y, request.z, request.orientation_x, request.orientation_y, request.orientation_z, request.orientation_w)
    except rospy.ROSInterruptException:
        return

    return GoalPoseJointSpaceResponse(success=success, goal_joints=position_joint_space)


def _on_endpoint_state(msg):
    """
        Callback function to get current end-point state
    """
    # pose message received is a vectorised column major transformation matrix
    global CARTESIAN_POSE
    cart_pose_trans_mat = np.asarray(msg.O_T_EE).reshape(4,4,order='F')

    CARTESIAN_POSE = {
        'position': cart_pose_trans_mat[:3,3],
        'orientation': quaternion.from_rotation_matrix(cart_pose_trans_mat[:3,:3]) }

def get_final_joint_config():
    rospy.init_node("move_group_python_interface", anonymous=True)
    cartesian_state_sub = rospy.Subscriber(
        'panda_simulator/custom_franka_state_controller/tip_state',
        EndPointState,
        _on_endpoint_state,
        queue_size=1,
        tcp_nodelay=True)

    service = rospy.Service('calc_ik', GoalPoseJointSpace, callback)
    rospy.loginfo("IK service started...")
    
    rospy.spin()

if __name__ == "__main__":
    try:
        get_final_joint_config()
    except rospy.ROSInterruptException:
        pass 