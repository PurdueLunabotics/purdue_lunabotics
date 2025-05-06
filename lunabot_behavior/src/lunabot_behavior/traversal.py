import rospy
import threading
from queue import Queue
import math
import random
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class TraversalManager:
    """
    Manages traversal, by keeping track of the current goal and republishing it when needed.
    """

    def __init__(self):
        """
        Start the manager. Note, this requires an already-running ROS node.
        """

        self.map: np.ndarray = None
        self.map_resolution: float = 0
        self.map_x_offset: int = -1
        self.map_y_offset: int = -1
        self.map_lock: threading.Lock = threading.Lock()

        self.robot_odom: Odometry = None

        self.current_goal: list[float] = None  # Goal in real-world coordinates

        self.OCCUPANCY_THRESHOLD: int = 50  # What value (greater than or equal to) is considered occupied


        self.frequency = 10  # Hz

        map_topic = rospy.get_param("/nav/map_topic", "/maps/costmap_node/global_costmap/costmap")
        map_update_topic = rospy.get_param("/nav/map_update_topic", "/maps/costmap_node/global_costmap/costmap_updates")
        odom_topic = rospy.get_param("/odom_topic")

        rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
        rospy.Subscriber(map_update_topic, OccupancyGridUpdate, self.map_update_callback)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber("/nav/global_path", Path, self.path_callback)

        self.goal_publisher = rospy.Publisher("/goal", PoseStamped, queue_size=1, latch=True)
        self.traversal_publisher = rospy.Publisher("/behavior/traversal_enabled", Bool, queue_size=1, latch=True)
        self.backwards_publisher = rospy.Publisher("/traversal/backwards", Bool, queue_size=1, latch=True)



    def traverse_to_goal(self, goal: PoseStamped, drive_backwards: bool = False):
        """
        Traverses to a given goal- blocks until the robot is within the threshold of the goal.
        """

        # wait for map
        while (self.map is None):
            rospy.sleep(0.1)

        self.current_goal = [goal.pose.position.x, goal.pose.position.y]

        self.check_and_replace_goal()

        self.goal_publisher.publish(goal)

        traversal_message = Bool()
        traversal_message.data = True
        self.traversal_publisher.publish(traversal_message)

        direction_message = Bool()
        direction_message.data = drive_backwards
        self.backwards_publisher.publish(direction_message)

        # Wait until the robot is close to the goal
        while not rospy.is_shutdown() and not self.is_close_to_goal():
            rospy.sleep(0.1)

        traversal_message.data = False
        self.traversal_publisher.publish(traversal_message)

    def stop_traversal(self):
        """
        Tells robot to stop traversing
        """

        traversal_message = Bool()
        traversal_message.data = False
        self.traversal_publisher.publish(traversal_message)

        self.current_goal = None

    def check_and_replace_goal(self):
        """
        Check if the goal is an obstacle given the current map/goal. If so, search for the closest non-occupied node, and republish it, so path planning will continue.
        """

        if (self.current_goal is None):
            return

        grid_coords = self.convert_to_grid(self.current_goal)

        if (self.map is None):
            return
        
        # Check out-of-bounds
        if (grid_coords[0] < 0 or grid_coords[1] < 0 or grid_coords[0] >= len(self.map) or grid_coords[1] >= len(self.map[0])):
            return

        # Check if not occupied
        if (self.map[grid_coords[0]][grid_coords[1]] < self.OCCUPANCY_THRESHOLD):
            return
        
        # If the goal is an obstacle, find a new one
        new_goal_coords = self.bfs_non_occupied(grid_coords)

        if (new_goal_coords is None):
            rospy.logerr("Traversal: Error in search for free node")
            return
        
        new_goal = self.convert_to_real(new_goal_coords)

        # if the new goal is the same as the old one, change it
        if (new_goal[0] == self.current_goal[0] and new_goal[1] == self.current_goal[1]):

            # loop until you find new coords that are both: -not an obstacle -different from the original
            while (self.map[new_goal_coords[0]][new_goal_coords[1]] > self.OCCUPANCY_THRESHOLD or
                   (new_goal[0] == self.current_goal[0] and new_goal[1] == self.current_goal[1])):
                
                # change randomly idk
                new_goal_coords[0] += random.randint(-1, 1)
                new_goal_coords[1] += random.randint(-1, 1)

                new_goal = self.convert_to_real(new_goal_coords)



        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = new_goal[0]
        goal.pose.position.y = new_goal[1]
        goal.pose.position.z = 0

        self.goal_publisher.publish(goal)
        rospy.loginfo("Traversal: Goal is an obstacle, new goal is: " + str(new_goal))
        self.current_goal = new_goal

    def is_close_to_goal(self) -> bool:
        """
        Checks if the robot is close to the current goal
        """

        THRESHOLD = 0.6 # meters

        x = self.robot_odom.pose.pose.position.x
        y = self.robot_odom.pose.pose.position.y

        goal_x = self.current_goal[0]
        goal_y = self.current_goal[1]

        distance = math.sqrt((x - goal_x)**2 + (y - goal_y)**2)

        rospy.logdebug("Behavior: Distance to goal: " + str(distance))

        return distance < THRESHOLD

    def bfs_non_occupied(self, current_node: list) -> 'list[int]':
        """
        Searches for the nearest non-occupied node closest to the given node. Used for finding a path if the robot is stuck on top of an obstacle.
        """

        nodequeue = Queue()
        visited_nodes = set()

        nodequeue.put(current_node)

        while not nodequeue.empty():

            node = nodequeue.get()

            if tuple(node) in visited_nodes:
                continue

            if  (0 <= node[0] < len(self.map) and 0 <= node[1] < len(self.map[0]) and
                self.map[node[0]][node[1]] < self.OCCUPANCY_THRESHOLD):
                return node

            directions = [[-1, 0], [1, 0], [0, -1], [0, 1]]  # above, below, left, right

            for direction in directions:
                new_node = [node[0] + direction[0], node[1] + direction[1]]
                # Add the node if in bounds
                if 0 <= new_node[0] < len(self.map) and 0 <= new_node[1] < len(self.map[0]):
                    nodequeue.put(new_node)

            visited_nodes.add(tuple(node))
        return None

        
    def convert_to_grid(self, position: 'list[float]') -> 'list[int]':
        """
        Convert a real world (x y) position to grid coordinates. Grid offset should be in the same frame as position, the grid is row-major.
        """

        shifted_pos = [position[0] - self.map_x_offset, position[1] - self.map_y_offset]
        coord = [
            int(shifted_pos[1] / self.map_resolution + 0.5),
            int(shifted_pos[0] / self.map_resolution + 0.5)
        ]

        return coord
    
    def convert_to_real(self, coord: 'list[int]') -> 'list[float]':
        """
        Convert a grid position to real world (x y) coordinates. Grid offset should be in the same frame as position, the grid is row-major.
        """

        position = [(coord[1] + 0.5) * self.map_resolution, (coord[0] + 0.5) * self.map_resolution]
        position = [position[0] + self.map_x_offset, position[1] + self.map_y_offset]
        return position

    def path_callback(self, msg: Path):
        if (len(msg.poses) == 0):
            if (self.current_goal is not None):
                self.check_and_replace_goal()

                goal = PoseStamped()
                goal.header.frame_id = "odom"
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = self.current_goal[0]
                goal.pose.position.y = self.current_goal[1]
                goal.pose.position.z = 0
        
                self.goal_publisher.publish(goal)

    def odom_callback(self, msg: Odometry):
        self.robot_odom = msg

    def map_callback(self, msg: OccupancyGrid):
        self.map_lock.acquire()

        data_arr = np.array(msg.data)

        width = msg.info.width
        height = msg.info.height
        self.map = np.reshape(data_arr, (height, width))

        self.map_resolution = msg.info.resolution
        self.map_x_offset = msg.info.origin.position.x
        self.map_y_offset = msg.info.origin.position.y

        if (np.all(data_arr == 0)): # If we get a blank map, we still use it as it will be followed by an update
            self.map_lock.release()
            return

        # check if the current goal is an obstacle, if so, replace it
        self.check_and_replace_goal()

        self.map_lock.release()

    def map_update_callback(self, msg: OccupancyGridUpdate):

        self.map_lock.acquire()

        data_arr = np.array(msg.data)

        if (self.map is None):
            self.map_lock.release()
            return

        if (np.all(data_arr == 0)):
            self.map_lock.release()
            return

        temp_map = self.map.copy()

        if (msg.y + msg.height >= len(temp_map) or msg.x + msg.width >= len(temp_map[0])):
            self.map_lock.release()
            return

        index = 0
        for i in range(msg.y, msg.y + msg.height):
            for j in range(msg.x, msg.x + msg.width):
                temp_map[i][j] = data_arr[index]
                index += 1

        self.map = temp_map.copy()

        # check if the current goal is an obstacle, if so, replace it
        self.check_and_replace_goal()

        self.map_lock.release()
