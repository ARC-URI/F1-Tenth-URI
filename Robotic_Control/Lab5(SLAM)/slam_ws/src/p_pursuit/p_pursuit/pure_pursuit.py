import csv
from math import atan2, cos, hypot, sin

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np


class pure_pursuit:
    def __init__(self):
        with open('centerline.csv', mode='r', encoding='utf-8') as file:
            reader = csv.reader(file)
            next(reader)
            self.path_points = [(float(row[0]), float(row[1])) for row in reader]

        self.K_dd = 1.0
        self.min_ld = 0.5
        self.max_ld = 2.0
        self.current_speed = 1.0
        self.wheelbase = 0.33
        self.current_position = (0.0, 0.0)
        self.current_orientation = 0.0
        self.closet_point = 0

    def update_position(self, position, orientation):
        self.current_position = position
        self.current_orientation = orientation

    def set_path(self, path_points):
        self.path_points = path_points
 #----------------Line-Circle Intersection with Bounds----------------#
 #----------------Credit to Purdue sig bots---------------------------#
    # helper function: sgn(num)
    # returns -1 if num is negative, 1 otherwise
    def sgn (num):  
        if num >= 0:
            return 1
        else:
            return -1
        
    # currentPos: [currentX, currentY]
    # pt1: [x1, y1]
    # pt2: [x2, y2]
    def line_circle_intersection (currentPos, pt1, pt2, lookAheadDis):

        # extract currentX, currentY, x1, x2, y1, and y2 from input arrays
        currentX = currentPos[0]
        currentY = currentPos[1]
        x1 = pt1[0]  
        y1 = pt1[1]
        x2 = pt2[0]  
        y2 = pt2[1]
        
        # boolean variable to keep track of if intersections are found
        intersectFound = False
        
        # output (intersections found) should be stored in arrays sol1 and sol2
        # if two solutions are the same, store the same values in both sol1 and sol2
        
        # subtract currentX and currentY from [x1, y1] and [x2, y2] to offset the system to origin
        x1_offset = x1 - currentX
        y1_offset = y1 - currentY
        x2_offset = x2 - currentX
        y2_offset = y2 - currentY  
        
        # calculate the discriminant using equations from the wolframalpha article
        dx = x2_offset - x1_offset
        dy = y2_offset - y1_offset
        dr = math.sqrt (dx**2 + dy**2)
        D = x1_offset*y2_offset - x2_offset*y1_offset
        discriminant = (lookAheadDis**2) * (dr**2) - D**2  
        
        # if discriminant is >= 0, there exist solutions
        if discriminant >= 0:
        
            # calculate the solutions
            sol_x1 = (D * dy + sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
            sol_x2 = (D * dy - sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
            sol_y1 = (- D * dx + abs(dy) * np.sqrt(discriminant)) / dr**2
            sol_y2 = (- D * dx - abs(dy) * np.sqrt(discriminant)) / dr**2
            
            # add currentX and currentY back to the solutions, offset the system back to its original position
            sol1 = [sol_x1 + currentX, sol_y1 + currentY]
            sol2 = [sol_x2 + currentX, sol_y2 + currentY]
            
            # find min and max x y values
            minX = min(x1, x2)
            maxX = max(x1, x2)
            minY = min(y1, y2)
            maxY = max(y1, y2)
            
            # check to see if any of the two solution points are within the correct range
            # for a solution point to be considered valid, its x value needs to be within minX and maxX AND its y value needs to be between minY and maxY
            # if sol1 OR sol2 are within the range, intersection is found
            if (minX <= sol1[0] <= maxX and minY <= sol1[1] <= maxY) or (minX <= sol2[0] <= maxX and minY <= sol2[1] <= maxY) :
                intersectFound = True 
                
                # now do a more detailed check to determine which point is valid, which is not
                if (minX <= sol1[0] <= maxX and minY <= sol1[1] <= maxY) :
                    print ('solution 1 is valid!') 
                if (minX <= sol2[0] <= maxX and minY <= sol2[1] <= maxY) :
                    print ('solution 1 is valid!')
            
        # graphing functions to visualize the outcome
        # ----------------------------------------------------------------------------------------------------------------------------------------
        plt.plot ([x1, x2], [y1, y2], '--', color='grey')
        draw_circle (currentX, currentY, lookAheadDis, 'orange')
        if intersectFound == False :
            print ('No intersection Found!')
        else:
            print ('Solution 1 found at [{}, {}]'.format(sol1[0], sol1[1]))
            print ('Solution 2 found at [{}, {}]'.format(sol2[0], sol2[1]))
            plt.plot (sol1[0], sol1[1], '.', markersize=10, color='red', label='sol1')
            plt.plot (sol2[0], sol2[1], '.', markersize=10, color='blue', label='sol2')
            plt.legend()
        
        plt.axis('scaled')
        plt.show()
        
    # now call this function and see the results!
    line_circle_intersection ([0, 1], [2, 3], [0, 1], 1)
#--------------------------------- End of Line-Circle Intersection with Bounds------#
    def path_point_distance(self, point1, point2):
        return hypot(point1[0] - point2[0], point1[1] - point2[1])

    def find_lookahead_point(self, speed):
        min_ld = 0.5  # Minimum lookahead distance
        max_ld = 2.0  # Maximum lookahead distance
        K_dd = 1.0  # Gain for lookahead distance adjustment



        for point in self.path_points:
            dis_to_point = self.path_point_distance(self.current_position, point)

        for target_point in self.path_points:
            dis_to_point = self.path_point_distance(self.current_position, target_point)
            l_d = np.clip(K_dd * speed, min_ld, max_ld)
            if dis_to_point >= l_d:
                return target_point
        return None

    def compute_steering_angle(self):
        lookahead_point = self.find_lookahead_point(self.current_speed)
        if lookahead_point is None:
            return None

        dx = lookahead_point[0] - self.current_position[0]
        dy = lookahead_point[1] - self.current_position[1]

        angle_to_point = atan2(dy, dx)
        
        angle_to_point = atan2(dy, dx)
        steering_angle = atan2(2 * self.wheelbase * sin(angle_to_point - self.current_orientation), hypot(dx, dy))
        return steering_angle

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        #sub block
        self.Lidar_sub = self.create_subscription(
            LaserScan,
            '/autodrive/f1tenth_1/lidar',
            self.lidar_callback,
            10)
        self.pose_sub = self.create_subscription(
            LaserScan,
            '/autodrive/f1tenth_1/pose',
            self.pose_callback,
            10)
        #timer block
        self.timer = self.create_timer(0.5, 
            self.timer_callback)
        
        self.Lidar_sub  
        self.pose_sub  
        
        self.pure_pursuit_instance = pure_pursuit()

    def timer_callback(self):
        
        self.pure_pursuit_instance.print_path_points()

    def lidar_callback(self, msg):
        0
        #self.get_logger().info('I heard: "%s"' % msg.ranges)

    def pose_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.pose.pose.position.x)
        self.get_logger().info('I heard: "%s"' % msg.pose.pose.position.y)
        self.get_logger().info('I heard: "%s"' % msg.pose.pose.orientation.z)
        self.pure_pursuit.update_position(
            position=(msg.pose.pose.position.x, msg.pose.pose.position.y),
            orientation=msg.pose.pose.orientation.z
        )


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    print("Subscriber node has been started. Listening to /autodrive/f1tenth_1/lidar topic...")
    print("Subscriber node has been started. Listening to /autodrive/f1tenth_1/pose topic...")
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()