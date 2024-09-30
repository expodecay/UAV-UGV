#! /usr/bin/env python
from __future__ import print_function
import actionlib
import rospy
import actionlib_tutorials.msg
import cpr_gps_navigation_msgs.msg
import geometry_msgs.msg
import utm
from robot_localization.srv import SetDatum
from geographic_msgs.msg import GeoPose
from robot_localization.srv import SetDatum
from geographic_msgs.msg import GeoPose
def find_utm_coords(lat, lon):
    u = utm.from_latlon(lat, lon)
    east = u[0]
    north = u[1]
    return north, east
def sendGoal():
#we chose datum at the following location
    datum_lat =  34.059132
    datum_lon = -117.821365
    rospy.wait_for_service('ekfs_initial_estimate', timeout = 2.0)
    try:
        rospy.loginfo("HEE")
        datum_service = rospy.ServiceProxy('ekfs_initial_estimate', SetDatum)
        req = GeoPose()
        req.position.latitude = datum_lat;
        req.position.longitude = datum_lon;
        req.orientation.w = 1.0;
        res = datum_service(req)
        rospy.loginfo("Datum set, sleeping for 2 seconds")
    except rospy.ServiceException, e:
        print("Service call failed")
        return
    #sleep to make sure datum is set properly and ekfs are converged
    rospy.sleep(2.0)
    #assume that our goal is to get to the goalpoint (43.5011, -80.5463) through the viapoints (43.5010, -80.5462) and (43.5008, -80.5463)
    datum_north, datum_east = find_utm_coords(datum_lat, datum_lon) #find utm coordinate of the
    datum
    goalpoint_lat = 34.059613
    goalpoint_lon = -117.820824
    goalpoint_north, goalpoint_east = find_utm_coords(goalpoint_lat, goalpoint_lon) #find utm coordinate of the desired point

    viapoint1_lat = 34.059183
    viapoint1_lon = -117.822027
    viapoint1_north, viapoint1_east = find_utm_coords(viapoint1_lat, viapoint1_lon) #find utm coordinate of the desired point

    viapoint2_lat = 34.058804
    viapoint2_lon = -117.821231
    viapoint2_north, viapoint2_east = find_utm_coords(viapoint2_lat, viapoint2_lon) #find utm coordinate of the desired point

    # Creates a SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('missionplan', cpr_gps_navigation_msgs.msg.
    MissionAction)
    # Waits until the action server has started up and started
    # listening for goals.
    if client.wait_for_server(timeout = rospy.Duration(2.0)):
    # Creates a goal to send to the action server.
        goal = cpr_gps_navigation_msgs.msg.MissionGoal()
        goal.mission.header.seq = 1;
        goal.mission.header.stamp = rospy.Time.now();

        goal.mission.header.frame_id = "map";
        goal.mission.goalpoint.x = goalpoint_east - datum_east;
        goal.mission.goalpoint.y = goalpoint_north - datum_north;
        goal.mission.goalpoint_theta = 0.0;

        viapoint1 = cpr_gps_navigation_msgs.msg.Waypoint();
        viapoint1.x = viapoint1_east - datum_east;
        viapoint1.y = viapoint1_north - datum_north;

        viapoint2 = cpr_gps_navigation_msgs.msg.Waypoint();
        viapoint2.x = viapoint2_east - datum_east;
        viapoint2.y = viapoint2_north - datum_north;

        goal.mission.viapoints = [viapoint2, viapoint1];
    # UNCOMMENT IF YOU WANT A FINAL HEADING AT THE GOAL POINT
    # goal.mission.set_final_heading = True
    # goal.mission.goalpoint_theta = 30
    # UNCOMMENT IF YOU WANT A FINAL GOAL TOLERANCE SET AT THE GOAL POINT [m, rad]
    # goal.mission.set_goal_tolerance = True
    # goal.mission.pose_tolerance = 0.1
    # goal.mission.yaw_tolerance = 0.2
    # Sends the goal to the action server.
        client.send_goal(goal)
    else:
        return False;
        # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Prints out the result of executing the action
    return client.get_result() # A FibonacciResult

if __name__ == '__main__':
    try:
        rospy.init_node('simple_nav_example')
        res = sendGoal()
        if res:
            print("mission completed!")
        else:
            print("mission server is not available!")
    except rospy.ROSInterruptException:
        print("mission server is not avialable")
        pass


