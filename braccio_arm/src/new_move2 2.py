from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):

    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        
        d = dist((x1, y1, z1), (x0, y0, z0))
        
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True
    
class MoveGroupPythonInterfaceTutorial(object):

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        
        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()
        
        group_name = "magician_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names



    def go_to_joint_home(self):
        
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)



    def go_to_joint_state(self):
        
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0.5
        joint_goal[2] = 0.8
        joint_goal[3] = 0

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def go_to_pose_goal(self):
        
        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.5
        pose_goal.position.x = 0
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0

        move_group.set_pose_target(pose_goal)

        plan = move_group.go(wait=True)
        
        move_group.stop()
        
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


    def plan_cartesian_path(self, scale = 1 ):
        
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z = 0.1  
        wpose.position.y = 0.1  
        waypoints.append(copy.deepcopy(wpose))

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0 )  

        return plan, fraction
        
    def plan_cartesian_path_step(self, i):
        
        if i%6 == 0 :
        
        	move_group = self.move_group

        	waypoints = []

        	wpose = move_group.get_current_pose().pose
        	wpose.position.z = 0.1  
        	wpose.position.y = 0.1  
        	waypoints.append(copy.deepcopy(wpose))

        	waypoints.append(copy.deepcopy(wpose))

        	(plan, fraction) = move_group.compute_cartesian_path(
            	waypoints, 0.01, 0.0 )  

        	return plan, fraction
        	
        elif ((i > 6) and (i%6 == 1)) :
        
	        tutorial = MoveGroupPythonInterfaceTutorial()
	        tutorial.detach_box()
	        tutorial.remove_box()
        	
        	move_group = self.move_group

        	waypoints = []

        	wpose = move_group.get_current_pose().pose
        	wpose.position.z = 0.1  
        	wpose.position.y = 0.1  
        	waypoints.append(copy.deepcopy(wpose))

        	waypoints.append(copy.deepcopy(wpose))

        	(plan, fraction) = move_group.compute_cartesian_path(
            	waypoints, 0.01, 0.0 )  
            	
        	return plan, fraction
        
        elif i%6 == 1 :
        	move_group = self.move_group

        	waypoints = []

        	wpose = move_group.get_current_pose().pose
        	wpose.position.z = 0.2  
        	wpose.position.y = 0.1 
        	waypoints.append(copy.deepcopy(wpose))

        	waypoints.append(copy.deepcopy(wpose))

        	(plan, fraction) = move_group.compute_cartesian_path(
            	waypoints, 0.01, 0.0 )  

        	return plan, fraction
        
        elif i%6 == 2 :
        	move_group = self.move_group

        	waypoints = []

        	wpose = move_group.get_current_pose().pose
        	wpose.position.z = 0.2  
        	wpose.position.y = 0.2 
        	waypoints.append(copy.deepcopy(wpose))

        	waypoints.append(copy.deepcopy(wpose))

        	(plan, fraction) = move_group.compute_cartesian_path(
            	waypoints, 0.01, 0.0 )  

        	return plan, fraction
        	        
        elif i%6 == 3 :
        	move_group = self.move_group

        	waypoints = []

        	wpose = move_group.get_current_pose().pose
        	wpose.position.z = 0.1  
        	wpose.position.y = 0.2  
        	waypoints.append(copy.deepcopy(wpose))

        	waypoints.append(copy.deepcopy(wpose))

        	(plan, fraction) = move_group.compute_cartesian_path(
            	waypoints, 0.01, 0.0 )  

        	return plan, fraction

        elif i%6 == 4 :
        
	        tutorial = MoveGroupPythonInterfaceTutorial()
	        tutorial.add_box()
	        tutorial.attach_box()
        	
        	move_group = self.move_group

        	waypoints = []

        	wpose = move_group.get_current_pose().pose
        	wpose.position.z = 0.2  
        	wpose.position.y = 0.2 
        	waypoints.append(copy.deepcopy(wpose))

        	waypoints.append(copy.deepcopy(wpose))

        	(plan, fraction) = move_group.compute_cartesian_path(
            	waypoints, 0.01, 0.0 )  

        	return plan, fraction
        	
        elif i%6 == 5 :
        	move_group = self.move_group

        	waypoints = []

        	wpose = move_group.get_current_pose().pose
        	wpose.position.z = 0.2  
        	wpose.position.y = 0.1 
        	waypoints.append(copy.deepcopy(wpose))

        	waypoints.append(copy.deepcopy(wpose))

        	(plan, fraction) = move_group.compute_cartesian_path(
            	waypoints, 0.01, 0.0 )  

        	return plan, fraction  
        	      	
    def display_trajectory(self, plan):
        
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)


    def execute_plan(self, plan):
        
        move_group = self.move_group

        move_group.execute(plan, wait=True)




    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        
        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        
        return False
        

    def add_box(self, timeout=4):
        
        box_name = self.box_name
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "link_6"
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.z = 0.0 
        box_pose.pose.position.y = -0.03
        box_pose.pose.position.x = -0.02
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.035, 0.035, 0.035))

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def attach_box(self, timeout=4):
        
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        grasping_group = "magician_arm"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        scene.remove_attached_object(eef_link, name=box_name)
        
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        
        box_name = self.box_name
        scene = self.scene

        scene.remove_world_object(box_name)

        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )



def main():
    try:
        print("")
        print("========================  moveit projects start ======================== ")
        print("")
        input(
            "============ Press `Enter` to begin setting up the moveit_commander ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()

        input(
            "============ Press `Enter` to execute a movement using a joint state home ...")
        tutorial.go_to_joint_home()
        
        
#        input("============ Press `Enter` to execute a movement using a joint state goal ...")
#        tutorial.go_to_joint_state()

#        input("============ Press `Enter` to execute a movement using a pose goal ...")
#        tutorial.go_to_pose_goal()


        input("============ Press `Enter` to run program ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path()
        tutorial.display_trajectory(cartesian_plan)
        tutorial.execute_plan(cartesian_plan)

        
        i = 0
        while i < 26 :
        
        	cartesian_plan, fraction = tutorial.plan_cartesian_path_step(i)
        	tutorial.display_trajectory(cartesian_plan)
        	tutorial.execute_plan(cartesian_plan)
        	i += 1

#------------------------------------

#        input("============ Press `Enter` to plan and display a Cartesian path ...")
#        cartesian_plan, fraction = tutorial.plan_cartesian_path()
#        input("============ Press `Enter` to display a saved trajectory ...")
#        tutorial.display_trajectory(cartesian_plan)
#        input("============ Press `Enter` to execute a saved path ...")
#        tutorial.execute_plan(cartesian_plan)
        
#        input("============ Press `Enter` to add a box to the planning scene ...")
#        tutorial.add_box()
#        input("============ Press `Enter` to attach a Box to the Magician ...")
#        tutorial.attach_box()     
#        input("============ Press `Enter` to detach the box from the Panda robot ...")
#        tutorial.detach_box()
#        input("============ Press `Enter` to remove the box from the planning scene ...")
#       tutorial.remove_box()

        print("============ Python tutorial demo complete!")
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
        
