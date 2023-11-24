#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

 
current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg

#current_position_sub의 callback함수    
cur_position = PoseStamped()    
def current_position(msg):
    global cur_position
    cur_position = msg

# 방문할 waypoint 설정    
waypointlist = [[5,0,10],[5,7,10],[5,7,8],[5,0,8],[5,0,6],[5,7,6],[5,7,4],[5,0,4],[5,0,2],[5,7,2],[0,0,0]]

if __name__ == "__main__":
    #waypoint
    
    
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    
    # 기체의 pose를 subscribe
    current_position_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = current_position)
    # 특정 position으로 가라는 topic을 publish 
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2
    
    

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    # Define a variable to keep track of the current waypoint index
    current_waypoint = 0
    
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
        
        pose.pose.position.x = waypointlist[current_waypoint][0]
        pose.pose.position.y = waypointlist[current_waypoint][1]
        pose.pose.position.z = waypointlist[current_waypoint][2]
        
        local_pos_pub.publish(pose)
        
        # Check if the drone is close to the current waypoint
        distance_to_waypoint = ((cur_position.pose.position.x - pose.pose.position.x) ** 2 +
                            (cur_position.pose.position.y - pose.pose.position.y) ** 2 +
                            (cur_position.pose.position.z - pose.pose.position.z) ** 2) ** 0.5
        
        # Define a threshold for reaching the waypoint
        waypoint_threshold = 0.15
        
        if distance_to_waypoint < waypoint_threshold:
            # Move to the next waypoint
            current_waypoint += 1
        
        # indexoutofrange 에러 발생 안시키려고
        if current_waypoint >= len(waypointlist):
            current_waypoint -= 1
        rate.sleep()