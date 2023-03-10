#! /usr/bin/env python

from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

import rospy
import cpr_gps_navigation_msgs.msg
import utm
import cpr_gps_navigation_msgs.srv

def find_utm_coords(lat, lon):
  u = utm.from_latlon(lat, lon)
  east = u[0]
  north = u[1]
  return north, east

def sendGoal():

    # we set the datum at the following location
    datum_lat = 34.059319
    datum_lon = -117.820521

    #assume that our goal is to get to the goalpoint (43.5011, -80.5463) through the viapoints (43.5010, -80.5462) and (43.5008, -80.5463)
    datum_north, datum_east = find_utm_coords(datum_lat, datum_lon) #find utm coordinate of the datum

    rospy.wait_for_service('/set_datum', timeout = 2.0)

    try:
        datum_service = rospy.ServiceProxy('/set_datum', cpr_gps_navigation_msgs.srv.TaskSrv)
        res = datum_service("", [datum_north, datum_east], [])
        rospy.loginfo("Datum set, sleeping for 2 seconds")
    except rospy.ServiceException as e:
        print("Service call failed")
        return

    #sleep to make sure datum is set properly and ekfs are converged
    rospy.sleep(2.0)

    # original code uses 4 decimal digits, we use 6. maybe that is the problem?
    goalpoint_lat = 34.059361    
    goalpoint_lon = -117.820990
    goalpoint_north, goalpoint_east = find_utm_coords(goalpoint_lat, goalpoint_lon) #find utm coordinate of the desired point

    viapoint1_lat = 34.059360
    viapoint1_lon = -117.821234
    viapoint1_north, viapoint1_east = find_utm_coords(viapoint1_lat, viapoint1_lon) #find utm coordinate of the desired point


    viapoint2_lat = 34.059223
    viapoint2_lon = -117.821100
    viapoint2_north, viapoint2_east = find_utm_coords(viapoint2_lat, viapoint2_lon) #find utm coordinate of the desired point


    # Creates a SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('missionplan', cpr_gps_navigation_msgs.msg.MissionAction)

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

        print(viapoint1)

        viapoint2 = cpr_gps_navigation_msgs.msg.Waypoint();
        viapoint2.x = viapoint2_east - datum_east;
        viapoint2.y = viapoint2_north - datum_north;


        goal.mission.viapoints = [viapoint2, viapoint1];

        # add final heading for goalpointT
        goal.mission.set_final_heading = True
        goal.mission.goalpoint_theta = 30

        # add final goal tolerance at the goal point [m, rad]
        goal.mission.set_goal_tolerance = True
        goal.mission.pose_tolerance = 0.1
        goal.mission.yaw_tolerance = 0.2


        # Sends the goal to the action server.
        client.send_goal(goal)
    else:
        return False;

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('simple_nav_example')
        res = sendGoal()
        if res:
            print("mission completed!")
        else:
            print("mission failed!")
    except rospy.ROSInterruptException:
        print("mission failed!")
        pass
