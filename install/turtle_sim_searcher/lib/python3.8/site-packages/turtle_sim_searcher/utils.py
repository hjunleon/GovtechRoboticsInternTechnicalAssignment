import time
import rclpy
import math

"""
Easy to access constants
"""
M_PI = math.pi
M_TWICE_PI = M_PI * 2
M_2_PI = M_PI / 2

"""
@brief Python implementation of getting current time in miliseconds
"""
def current_milli_time():
    return round(time.time() * 1000)

"""
@brief ROS implementation of getting current time in miliseconds
"""
def ros_time():
    return rclpy.time.Time().to_msg()

"""
@brief Helper function to pause the loop execution. 
@param seconds The duration to sleep in seconds
"""
def pause(seconds):
    time.sleep(seconds)

"""
@brief Helper function to remap -PI to PI to 0 to 2PI radians.
@param rad Radians in range of -PI to PI
"""
def remap_rad(rad):
    if rad < 0:
        toRet = (M_TWICE_PI + rad) % M_TWICE_PI
        return toRet
    
    return rad % M_TWICE_PI

"""
@brief Sqaure of eucliean distance
@param p1 Point ROS message
@param p2 Point ROS message
"""
def getRoughEuclideanDistance(p1,p2):
    return (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2

"""
@brief Calculates eucliean distance using two points
@param p1 Point ROS message
@param p2 Point ROS message
"""
def getEuclideanDistance(p1,p2):
    return math.sqrt(getRoughEuclideanDistance(p1,p2))