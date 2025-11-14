#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
#import cv2
#import base64
#from io import BytesIO
from langchain.agents import tool
#from langchain.llms import OpenAI 
#bridge = CvBridge()
#from remembr.agents.remembr_agent import ReMEmbRAgent
#from remembr.memory.milvus_memory import MilvusMemory
import requests
import rospy

# Set Pub/Sub
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

# ReMEmbRAgent Primitives
#rospy.loginfo("Initializing ReMEmbR Agent and Milvus Memory...")
#try:
    # Memory setup
#    memory = MilvusMemory("test_collection", db_ip='127.0.0.1', db_port=19530)
    # Uncomment only if we want to set the robot in "Exploration Mode" and clear previous memory entries
    # memory.reset() 

    # Initialize the agent (as you already had)
#    ltm_agent = ReMEmbRAgent(llm_type='gpt-4')
    # Give the agent access to the memory
#    ltm_agent.set_memory(memory)
#    rospy.loginfo("ReMEmbR Agent and Milvus Memory successfully initialized.")

#except Exception as e:
#    rospy.logerr(f"Failed to initialize ReMEmbR Agent: {e}")
#    ltm_agent = None
    # If the ltm_agent fails to initialize, this session will run without LTM capabilities.

#memory = MilvusMemory("test_collection", db_ip='127.0.0.1', db_port=19530)
#ltm_agent = ReMEmbRAgent(llm_type='gpt-4')


@tool
def move_waffle(linear: float = 0.2, angular: float = 0.0, duration: float = 2.0):
    """Move the TurtleBot3 Waffle with given linear and angular velocities."""
    vel = Twist()
    vel.linear.x = linear
    vel.angular.z = angular
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)
    while (rospy.Time.now() - start_time).to_sec() < duration:
        cmd_vel_pub.publish(vel)
        rate.sleep()
    cmd_vel_pub.publish(Twist())  # stop
    return f"Moved with linear={linear}, angular={angular} for {duration} sec."


@tool
def get_waffle_pose():
    """Get the robot's current position from /odom."""
    try:
        msg = rospy.wait_for_message("/odom", Odometry, timeout=5)
        pos = msg.pose.pose.position
        return f"Robot is at x={pos.x:.2f}, y={pos.y:.2f}."
    except rospy.ROSException:
        return "Odometry data not available."

@tool
def query_long_term_memory(query: str):
    """
    Queries the robot's long-term memory (Vector DB) to find the position of an object or location.
    Use this when you need to find something the robot has seen before, like "where is the desk?" or "find the fridge".
    The query should be a natural language question about the location of something.
    """
    rospy.loginfo(f"Querying LTM with: '{query}'")
    
    #if ltm_agent is None:
    #    rospy.logerr("LTM agent is not initialized. Cannot query memory.")
    #    return "Error: Long-term memory agent failed to initialize. Cannot query memory in this session"
        
    #try:
        # This is the same call from your test script
    #    response = ltm_agent.query(query)
        
        # Format the response for the main ROSA agent
    #    if response.position:
    #        pos = response.position
    #        # Return a useful string for the main agent
    #        return f"Memory query successful. Location: {response.text}. Position: [x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}]."
    #    else:
    #        return f"Memory query successful, but no specific position was found. Details: {response.text}"
   # 
   # except Exception as e:
   #     rospy.logerr(f"Failed to query LTM: {e}")
    #    return f"Error querying long-term memory: {e}"

    try:
        resp = requests.post("http://localhost:8000/query", json={"query": query}, timeout=10)
        if resp.status_code != 200:
            return f"Error from Remembr server: {resp.text}"
        data = resp.json()
        pos = data.get("position")
        if pos:
            rospy.loginfo(f"The Retrieved position was: '{pos}'")
            return f"Memory query successful. Location: {data['text']}. Position: [x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}]."
        else:
            rospy.loginfo(f"No position found in response. Details: '{data['text']}'")
            return f"Memory query successful, but no position found. Details: {data['text']}"
    except Exception as e:
        rospy.logerr(f"Failed to query Remembr server: {e}")
        return f"Error querying Remembr server: {e}"
