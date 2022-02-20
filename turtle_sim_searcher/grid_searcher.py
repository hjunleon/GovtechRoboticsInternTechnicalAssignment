import rclpy
from rclpy.node import Node
import time
import math
import turtle_sim_searcher.utils

#importing messages
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose

"""
Min and Max values for grid search to occur properly when the turtle starts from (1,1)
"""
MAX_LEN = 11.0 #1.0#1.5
MIN_LEN = 0.1

"""
Easy to access constants
"""
M_PI = math.pi
M_TWICE_PI = M_PI * 2
M_2_PI = M_PI / 2

"""
The most apt to prevent significant overshoots.
"""
ROS_RATE = 10000#10000  #100, 1000

"""
Kp and Ki are PID constants. Ki is inversely proportional to ROS_RATE 
so that the sum of error is not as affected by ROS loop frequency,
hence more accurate
"""
Kp = 3.25#1.5
Ki = 0.5 / ROS_RATE  #0.02#0.02


"""
Tolerances suitable for this ROS_RATE
"""
DIST_TOLERANCE = 50 / ROS_RATE #0.005   #0.1  #0.05   #0.075
ANGLE_TOLERANCE = 50 / ROS_RATE #0.005 #0.00005



"""
@brief Class that controls and displays information from the turtle sim..
"""
class turtle_controller(Node):
    """
    @brief Initialises the class. Establish relevant publishers and subscribers. Sets default/initial values
    for class attributes 
    @param w Width of grid search, default 1.5
    @param l length of grid search, default 9.0
    """
    def __init__(self, w=1.5, l=9.0):
        super().__init__('grid_planner')
        self.cmd_publisher = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.create_rate(ROS_RATE) 
        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.set_pose,
            10
        )
        
        self.width = w
        self.length = l
        self.dest = Point()
        self.dest.x = 0.0
        self.dest.y = 0.0
        self.mid = {
            'x': 5.5,
            'y': 5.5
        }
        self.cur_pose = None
        self.E = 0.0   # Strictly for PID sum-of-error component. Is reset for every complete motion

    """
    @brief Get grid search y-axis motion
    """
    def get_width(self):
        return self.width

    """
    @brief Set grid search y-axis motion
    """
    def set_width(self, width):
        self.width = width

    """
    @brief Get grid search x-axis motion
    """
    def get_length(self):
        return self.length

    """
    @brief Set grid search x-axis motion
    """
    def set_length(self, length):
        self.length = length


    """
    @brief Set self.dest to start position where grid search would begin
    """
    def init_start_pos(self): 
        # self.dest.x = self.mid['x'] - (self.length / 2)
        # self.dest.y = self.width
        self.dest.x = 1.0
        self.dest.y = 1.0

    """
    @brief Get latest pose of turtle sim
    """
    def get_pose(self):
        return self.cur_pose

    """
    @brief Set pose of turtle sim, used as subscriber callback
    """
    def set_pose(self, p):
        self.cur_pose = p

    """
    @brief Get latest turtle's headway angle
    """
    def get_pose_angle(self):
        return self.cur_pose.theta

    """
    @brief Helper function to reset the sum of error value.
    It is called after a motion is complete.
    """
    def reset_error_sum(self):
        self.E = 0

    """
    @brief Get angle turtle should turn towards
    """
    def get_dest_angle(self): 
        return self.dest.z

    """
    @brief Dest angle is stored in z of dest Point message
    @ang Angle of -PI to PI range in radians
    """
    def set_dest_angle(self, ang): 
        self.dest.z = ang

    """
    @brief Helper function to increment the destination rotational angle. 
    @ang Angle of -PI to PI range in radians
    """
    def inc_dest_angle(self, ang):
        self.dest.z += ang

    """
    @brief Helper function to increment the destination position. 
    Decrements are done with passing negative values for x and y
    @param x x-axis translation amount
    @param y y-axis translation amount
    """
    def inc_dest_pos(self, x,y):
        self.dest.x += x
        self.dest.y += y

    """
    @brief get dest position attribute
    """
    def get_dest_pos(self):
        return self.dest

    """
    @brief sets dest position attribute
    @param x x-axis coordinate
    @param y y-axis coordinate
    """
    def set_dest_pos(self, x,y):
        self.dest.x = x
        self.dest.y = y

    """
    
    @brief Generate twist to publish to turtle based on current pose
     and euclidean distance from destination
    
    @param isJustRotate boolean to determine if it's a on-the-spot rotation or 
        translation + rotation to dest position
    """
    def generate_twist(self, isJustRotate=False):
        if not self.cur_pose:
            return None

        # Calculate required linear velocity
        v_x = 0.0
        target_ang = 0.0
        if isJustRotate == False:
            e = turtle_sim_searcher.utils.getEuclideanDistance(self.dest, self.cur_pose)
            self.E += e

            # PID formula to compute v_x
            v_x = Kp * e + Ki * self.E   
            target_ang = math.atan2(self.dest.y - self.cur_pose.y,self.dest.x - self.cur_pose.x) 
        else:
            target_ang = self.get_dest_angle()
        
        # Calculate required rotational velocity
        current_ang = self.get_pose_angle()
        turn = target_ang - current_ang
        # Normalise to angle that's not a turn larger than 180 degrees
        if turn > M_PI:
            turn -= M_TWICE_PI
        elif turn < -M_PI:
            turn += M_TWICE_PI

        # PID formula to compute v_ang
        v_ang = 2.75 * Kp * turn
        return self.create_twist( v_x ,0.0, v_ang ) 

   

    """
    @brief Applied to rotations as it's more important for angle accuracy and it's position doesn't change
    """
    def is_angle_tolerable(self):
        return abs(self.get_dest_angle() -  self.get_pose_angle()) < ANGLE_TOLERANCE 


    """
    @brief Applied to translations (forward movements) as distance is a sufficient criteria.
    Using angle tolerance to get to any point on top of distance tolerance is difficult.
    """
    def is_distance_tolerable(self):
        return turtle_sim_searcher.utils.getEuclideanDistance(self.dest, self.get_pose()) < DIST_TOLERANCE 

    """
    @brief Combines the boolean statement to something that deals with either angle or distance tolerance 
    based on whether it's rotating on the spot or not

    @param isJustRotate boolean to determine if it's a on-the-spot rotation or 
        translation + rotation to dest position
    """
    def is_tolerable(self, isJustRotate):
        return ((isJustRotate == True) and self.is_angle_tolerable()) or ((isJustRotate == False) and self.is_distance_tolerable())

    """
    @brief A complete motion that moves the turtle to (self.dest.x, self.dest.y) if isJustRotate == False
    or rotate to self.dest.z if isJustRotate == True
    Restricts movement in turtle's local x-axis and angular steering, no local y-axis movement. 

    @param isJustRotate boolean to determine if it's a on-the-spot rotation or 
        translation + rotation to dest position
    """
    def move(self, isJustRotate=False):
        # print(f"isJustRotate: {isJustRotate}")
        self.reset_error_sum()
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.get_pose() == None:
                turtle_sim_searcher.utils.pause(1)
                continue
            twist = self.generate_twist(isJustRotate)
            is_tolerable = self.is_tolerable(isJustRotate)
            # print(f"is_tolerable: {is_tolerable}")
            if not twist: 
                turtle_sim_searcher.utils.pause(1)
            elif is_tolerable:
                self.stopbot()
                break
            else:
                self.publish_twist(twist)

    """
    @brief A set of complete motions that set turtle in the position and heading angle
    for grid search to proceed.
    """
    def go_to_start(self):
        self.move(False)
        time.sleep(0.5)
        self.set_dest_angle(0.0)
        self.move(True)

    """
    @brief Makes turtle conduct a grid search until it reaches the boundary of window.
    Repeats a set of complete motions in this order: 
        horizontal -> rotate -> vertical -> rotate

    A change of direction is achieved by negating the angle and length increment values.
    """
    def grid_search(self):
        # print("grid_search")
        isRight = True
        while (True):
            angle_inc = M_2_PI
            len_inc = self.length
            wid_inc = self.width
            if not isRight:
                angle_inc = -angle_inc
                len_inc = -len_inc

            self.inc_dest_pos(len_inc,0.0)
            print("horizontal motion")
            self.move(False)
            turtle_sim_searcher.utils.pause(0.5)
            # Criteria to check if reached boundary and can't explore further
            print(self.cur_pose.y + self.width)
            if (self.cur_pose.y + self.width > MAX_LEN): 
                break
            self.inc_dest_angle(angle_inc)
            print("%.4f" %(abs(self.get_pose_angle())))
            print("%.4f" %(M_2_PI))
            self.move(True)
            turtle_sim_searcher.utils.pause(0.5)
            self.inc_dest_pos(0.0,wid_inc)
            print("vertical motion")
            self.move(False)
            turtle_sim_searcher.utils.pause(0.5)
            self.inc_dest_angle(angle_inc)
            print("%.4f"%(abs(self.get_pose_angle())))
            print("%.4f"%(M_2_PI))
            self.move(True)
            turtle_sim_searcher.utils.pause(0.5)
            isRight = not isRight            
            
        
    """
    @brief Helper function to create a ROS Twist message

    @param x x-axis coordinate
    @param y y-axis coordinate
    @param angle angle in radians in range -PI to PI
    """
    def create_twist(self, x, y, angle):
        # self.get_logger().info("Creating twist")
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = angle
        return twist


    """
    @brief Helper function to publish a ROS Twist message
    @param twist Twist message to publish
    """
    def publish_twist(self, twist):
        # self.get_logger().info("Publishing twist")
        self.cmd_publisher.publish(twist)

    """
    @brief Helper function to publish a stop command.
    """
    def stopbot(self):
        # self.get_logger().info("Immediate Halt")
        twist = self.create_twist(0.0,0.0,0.0)
        self.publish_twist(twist)

"""
@brief To check if value given is a number
@param val A variable to check if is a number
"""
def is_num(val):
    try:
        float(val)
        return True
    except Exception as e:
        print(e)
        return False
    return False


"""
@brief To get user input for length and width. 
If length or width exceeds the dimmensions of the turlte sim window,  will set them
to min or max values required for grid search. 
"""
def get_input():
    width,length = None, None
    while (length == None):
        length = input("What is the grid length ? ")
        isValid = is_num(length)
        if not isValid:
            print("Length must be a integer or float")
            length = None
    while (width == None):
        width = input("What is the grid width ? ")
        isValid = is_num(width)
        if not isValid:
            print("Width must be a integer or float")
            width = None
    return min(max(float(width),MIN_LEN), MAX_LEN - 1.5)  , min(max(float(length),MIN_LEN), MAX_LEN - 1.5)



"""
@brief Function that sets up turtle sim controller and execute higher level motion.
"""
def run_turtle():
    w, l = get_input()
    print("angle is zero")
    tc = turtle_controller(w,l)
    try:
        tc.init_start_pos()
        tc.go_to_start()
        turtle_sim_searcher.utils.pause(1)
        tc.grid_search()
    except Exception as e:
        print(e)
    finally:
        # stop moving
        tc.stopbot()

"""
@brief Main function
@param args Command line arguments
"""
def main(args=None):
    rclpy.init(args=args)
    run_turtle()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
