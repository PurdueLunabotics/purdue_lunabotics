import rospy

from lunabot_msgs.msg import RobotEffort, RobotSensors
import interrupts
from lunabot_control.scripts.pid_controller import VelocityPIDController
from lunabot_control.scripts.clamp_output import clamp_output


class Excavate:
    '''
    A state used to autonomously excavate. Plunge lowers linear actuators while spinning the buckets,
    and excavate drives forwards/spins the buckets to try and meet a target depth of cut.
    '''
    
    def sensors_callback(self, msg: RobotSensors):
        self.robot_sensors = msg

    def __init__(self, effort_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if effort_publisher is None:
            self.effort_publisher: rospy.Publisher = rospy.Publisher("/effort", RobotEffort, queue_size=1, latch=True)
            rospy.init_node('plunge_node')
        else:
            self.effort_publisher: rospy.Publisher = effort_publisher

        self.robot_sensors = RobotSensors()

        rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

        self.rate = rospy.Rate(10)

        # 90 percent of max speed
        TARGET_EXCAVATION_VELOCITY = 127 * 0.9

        self.excavation_pid_controller = VelocityPIDController(TARGET_EXCAVATION_VELOCITY, 1, 0, 0, 1) #TODO find values

        self.left_drivetrain_pid_controller = VelocityPIDController(1, 1, 0, 0, 1) #TODO find values
        self.right_drivetrain_pid_controller = VelocityPIDController(1, 1, 0, 0, 1) #TODO find values
        
        # Constants (in meters)
        self.TARGET_DEPTH_OF_CUT = 0.003175 # Currently set to 1/8 inch, TODO tune this

        self.BUCKET_RADIUS = 0.0948
        self.BUCKET_SPACING = 0.0853 

        self.load_cell_weight_threshold = 0 #TODO find value
        self.max_lin_act_vel = 1 #TODO find value

        self.lin_act_curr_threshold = 10 # TODO find value

        self.excavation_current_threshold = 25 # Amps; TODO find/confirm value
    
    def excavate(self):
        self.plunge()
        self.trench()

    def plunge(self):
        """
        Controls plunging (moving linear actuators down + spinning excavation)
        """

        time.sleep(0.1) #Why is time.sleep() here. TODO investigate

        effort_message = RobotEffort()

        excavation_ang = self.robot_sensors.exc_ang #TODO have firmware give excavation angle directly?
        time = rospy.get_time()

        # Until the linear actuators reach the end (have high current), keep moving them down
        while (self.robot_sensors.act_right_curr < self.lin_act_curr_threshold): #TODO do proper method for end stop detection

            if (interrupts.check_for_interrupts() != interrupts.Errors.FINE):
                return False
            
            new_time = rospy.get_time()
            new_excavation_ang = self.robot_sensors.exc_ang

            dt = new_time - time

            # Set excavation based on the PID controller
            excavation_velocity = (new_excavation_ang - excavation_ang) / dt #TODO check calculations are good

            excavation_control = self.excavation_pid_controller.update(excavation_velocity, dt)
            effort_message.excavate = clamp_output(excavation_control)

            # Set the target linear actuator velocity to target the depth of cut
            target_actuator_velocity = self.TARGET_DEPTH_OF_CUT * excavation_velocity * self.BUCKET_RADIUS / self.BUCKET_SPACING

            effort_message.lin_act = clamp_output(127 / self.max_lin_act_vel * target_actuator_velocity) #No encoders so cannot do PID, estimating (will slighly underestimate on lower voltage)

            self.effort_publisher.publish(effort_message)

            # Check for excavation getting stuck (high current)
            if (self.robot_sensors.exc_curr > self.excavation_current_threshold):
                self.spin_excavation_backwards()

            time = new_time
            excavation_ang = new_excavation_ang

            self.rate.sleep()

        effort_message.lin_act = 0

        self.effort_publisher.publish(effort_message)

        return True

    def trench(self):
        """
        Controls trenching (spinning excavation + driving forward)
        """

        time.sleep(0.1) #Why is time.sleep() here. TODO investigate

        effort_message = RobotEffort() 

        excavation_ang = self.robot_sensors.exc_ang #TODO have firmware give excavation angle directly?
        time = rospy.get_time()

        load_cell_weight = self.robot_sensors.load_cell_weights[0] + self.robot_sensors.load_cell_weights[1]

        # TODO add logic for bucket stall
        # TODO add logic for stopping if obstacles exist (both rocks and craters)

        # Until the load cells are full (enough soil), keep moving the robot forward and spinning excavation
        while (load_cell_weight < self.load_cell_weight_threshold): #TODO do proper method for end stop detection

            if (interrupts.check_for_interrupts() != interrupts.Errors.FINE):
                return False
            
            new_time = rospy.get_time()
            new_excavation_ang = self.robot_sensors.exc_ang

            dt = new_time - time

            excavation_velocity = (new_excavation_ang - excavation_ang) / dt #TODO check calculations are good

            excavation_control = self.excavation_pid_controller.update(excavation_velocity, dt)
            effort_message.excavate = clamp_output(excavation_control)

            target_linear_vel = self.TARGET_DEPTH_OF_CUT * excavation_velocity * self.BUCKET_RADIUS / self.BUCKET_SPACING

            self.left_drivetrain_pid_controller.set_setpoint(target_linear_vel)
            left_drivetrain_ctrl = self.left_drivetrain_pid_controller.update(self.robot_sensors.drive_left_vel)
            effort_message.left_drive = clamp_output(left_drivetrain_ctrl)

            self.right_drivetrain_pid_controller.set_setpoint(target_linear_vel)
            right_drivetrain_ctrl = self.right_drivetrain_pid_controller.update(self.robot_sensors.drive_right_vel)
            effort_message.right_drive = clamp_output(right_drivetrain_ctrl)

            self.effort_publisher.publish(effort_message)

            # Check for excavation getting stuck (high current)
            if (self.robot_sensors.exc_curr > self.excavation_current_threshold):
                self.spin_excavation_backwards()

            time = new_time
            excavation_ang = new_excavation_ang

            load_cell_weight = self.robot_sensors.load_cell_weights[0] + self.robot_sensors.load_cell_weights[1]

            self.rate.sleep()

        effort_message.excavate = 0
        effort_message.left_drive = 0
        effort_message.right_drive = 0

        self.effort_publisher.publish(effort_message)

        return True
    
    def spin_excavation_backwards(self):
        """
        Spins excavation backwards for 1 second. Used to unstick rocks from buckets (if excavation gets high current)
        """

        # 90% of max speed, backwards
        EXCAVATION_SPEED = -127 * 0.9

        effort_message = RobotEffort()

        effort_message.excavate = EXCAVATION_SPEED
        self.effort_publisher.publish(effort_message)

        rospy.sleep(1)

        effort_message.excavate = 0
        self.effort_publisher.publish(effort_message)



    
if __name__ == "__main__":
    excavate_module = Excavate()
    excavate_module.excavate()