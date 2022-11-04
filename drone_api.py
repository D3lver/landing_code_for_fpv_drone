import rospy
from cv2 import Rodrigues
from geometry_msgs.msg import PoseStamped, Twist
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from threading import Thread
from math import pi, atan2, sqrt, sin, cos
from pygeodesy.geoids import GeoidPGM


class Drone_api:
    def __init__(self, allowable_error: float = 0.5,
                 # Переопределять нулевую точку после каждого start()
                 redefine_zero_point: bool = False,
                 # OFFBOARD включается не сам, а через RC или QGC (сначала нужно выполнить start())
                 offb_switch: bool = False,
                 # ROS не перехватывает Ctrl+C
                 disable_signals: bool = False):
        # GENERAL attributes
        self.__started = False
        self.__type_of_move = 'LOCAL_POSE'  # 'LOCAL_POSE' or 'GLOBAL_POSE' or 'VELOCITY'
        self.__current_state = State()
        self.__thread_command = None
        self.__yaw_head_first = False
        self.__offb_switch = offb_switch

        # LOCAL_POSE attributes
        self.__allowable_error = allowable_error  # m
        self.__redefine_zero_point = redefine_zero_point
        self.__start_xyz = (0, 0, 0)  # None
        self.__current_local_pose = None
        self.__last_command_local_pose = None

        # GLOBAL_POSE attributes
        self.__egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm',
                                kind=-3)
        self.__start_alt = None
        self.__current_global_pose = None
        self.__last_command_global_pose = None

        # VELOCITY attributes
        self.__last_command_vel = Twist()

        rospy.init_node('drone_offb', anonymous=True,
                        disable_signals=disable_signals)
        # get state, position, set position, velocity, mode and arm
        self.__state_sub = rospy.Subscriber('mavros/state', State,
                                            self.__state_cb, queue_size=10)
        self.__local_pos_sub = rospy.Subscriber('/mavros/local_position/pose',
                                                PoseStamped, self.__local_pos_cb, queue_size=10)
        self.__global_pos_sub = rospy.Subscriber('/mavros/global_position/global',
                                                 NavSatFix, self.__global_pos_cb, queue_size=10)
        self.__local_pos_pub = rospy.Publisher('mavros/setpoint_position/local',
                                               PoseStamped, queue_size=10)
        self.__global_pos_pub = rospy.Publisher('mavros/setpoint_position/global',
                                                GeoPoseStamped, queue_size=10)
        self.__local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped',
                                               Twist, queue_size=10)
        self.__set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.__arming_client = rospy.ServiceProxy('mavros/cmd/arming',
                                                  CommandBool)

    def __state_cb(self, state):
        self.__current_state = state

    def __local_pos_cb(self, pose):
        self.__current_local_pose = pose

    def __global_pos_cb(self, pose):
        self.__current_global_pose = pose

    def __command_target(self):
        rate = rospy.Rate(20)
        # Waiting for communication between MAVROS and autopilot
        while not rospy.is_shutdown() and not self.__current_state.connected:
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass
        last_request = rospy.Time.now()
        while self.__started and not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                # OFFBOARD and ARM
                if now - last_request > rospy.Duration(3):
                    last_request = now
                    if self.__offb_switch:
                        if self.__current_state.mode == 'OFFBOARD' and not self.__current_state.armed:
                            self.__arming_client(True)
                    else:
                        if self.__current_state.mode != 'OFFBOARD':
                            self.__set_mode_client(base_mode=0,
                                                   custom_mode='OFFBOARD')
                        elif not self.__current_state.armed:
                            # fixme: дрон бесконечно дизармится, если не взлететь
                            self.__arming_client(True)
                # CONTROL
                if self.__type_of_move == 'LOCAL_POSE':
                    delta_x = (self.__last_command_local_pose.pose.position.x
                               - self.__current_local_pose.pose.position.x)
                    delta_y = (self.__last_command_local_pose.pose.position.y
                               - self.__current_local_pose.pose.position.y)
                    distance = sqrt((delta_x)**2 + (delta_y)**2)
                    if self.__yaw_head_first and distance > self.__allowable_error:
                        yaw = atan2(delta_y, delta_x)
                        q = quaternion_from_euler(0, 0, yaw)
                        self.__last_command_local_pose.pose.orientation.x = q[0]
                        self.__last_command_local_pose.pose.orientation.y = q[1]
                        self.__last_command_local_pose.pose.orientation.z = q[2]
                        self.__last_command_local_pose.pose.orientation.w = q[3]
                    self.__local_pos_pub.publish(
                        self.__last_command_local_pose)
                elif self.__type_of_move == 'GLOBAL_POSE':
                    delta_x = (self.__last_command_local_pose.pose.position.x
                               - self.__current_local_pose.pose.position.x)
                    delta_y = (self.__last_command_local_pose.pose.position.y
                               - self.__current_local_pose.pose.position.y)
                    distance = sqrt((delta_x)**2 + (delta_y)**2)
                    if self.__yaw_head_first and distance > self.__allowable_error:
                        yaw = atan2(delta_y, delta_x)
                        q = quaternion_from_euler(0, 0, yaw)
                        self.__last_command_global_pose.pose.orientation.x = q[0]
                        self.__last_command_global_pose.pose.orientation.y = q[1]
                        self.__last_command_global_pose.pose.orientation.z = q[2]
                        self.__last_command_global_pose.pose.orientation.w = q[3]
                    self.__global_pos_pub.publish(
                        self.__last_command_global_pose)
                elif self.__type_of_move == 'VELOCITY':
                    self.__local_vel_pub.publish(self.__last_command_vel)
                rate.sleep()
            except rospy.exceptions.ROSException:
                pass

    # GENERAL methods
    def start(self):
        if not self.__started:
            # LOCAL
            while not rospy.is_shutdown() and self.__current_local_pose == None:
                rospy.sleep(0.2)
            self.__last_command_local_pose = PoseStamped()
            self.__last_command_global_pose = GeoPoseStamped()
            list_x = 0
            list_y = 0
            list_z = 0
            list_alt = 0
            for i in range(10):
                list_x += self.__current_local_pose.pose.position.x
                list_y += self.__current_local_pose.pose.position.y
                list_z += self.__current_local_pose.pose.position.z
                rospy.sleep(0.2)
            if self.__redefine_zero_point:
                self.__start_xyz = (list_x/10, list_y/10, list_z/10)
            else:
                self.__start_xyz = (0, 0, 0)
            # First "last_command"
            self.__last_command_local_pose.pose.position.x = self.__start_xyz[0]
            self.__last_command_local_pose.pose.position.y = self.__start_xyz[1]
            self.__last_command_local_pose.pose.position.z = self.__start_xyz[2] - 1
            self.__last_command_local_pose.pose.orientation.w = 1  # correct quaternion
            if self.__current_global_pose != None:
                for i in range(10):
                    list_alt += self.__current_global_pose.altitude
                    rospy.sleep(0.2)
                self.__start_alt = list_alt/10
                self.__last_command_global_pose.pose.position.latitude = self.__current_global_pose.latitude
                self.__last_command_global_pose.pose.position.longitude = self.__current_global_pose.longitude
                self.__last_command_global_pose.pose.orientation.w = 1

            self.__started = True
            if self.__thread_command is None or not self.__thread_command.is_alive():
                self.__thread_command = Thread(target=self.__command_target,
                                               daemon=True)
                self.__thread_command.start()
            rospy.sleep(1)

    def stop(self):
        self.__start_alt = None
        self.__started = False

    @staticmethod
    def sleep(time: float):
        try:
            rospy.sleep(time)
        except rospy.ROSInterruptException:
            pass

    @staticmethod
    def is_shutdown():
        return rospy.is_shutdown()

    # LOCAL_POSE methods
    @property
    def allowable_error(self):
        return self.__allowable_error

    @allowable_error.setter
    def allowable_error(self, distance: float):
        self.__allowable_error = distance

    @property
    def redefine_zero_point(self):
        return self.__redefine_zero_point

    @redefine_zero_point.setter
    def redefine_zero_point(self, redefine: bool):
        self.__redefine_zero_point = redefine

    def get_local_pose(self):
        x = self.__current_local_pose.pose.position.x + self.__start_xyz[0]
        y = self.__current_local_pose.pose.position.y + self.__start_xyz[1]
        z = self.__current_local_pose.pose.position.z + self.__start_xyz[2]
        angels = euler_from_quaternion([self.__current_local_pose.pose.orientation.x,
                                        self.__current_local_pose.pose.orientation.y,
                                        self.__current_local_pose.pose.orientation.z,
                                        self.__current_local_pose.pose.orientation.w])
        roll = angels[0]
        pitch = angels[1]
        yaw = angels[2]
        return x, y, z, roll, pitch, yaw

    def set_local_pose(self, x: float = None, y: float = None, z: float = None,
                       yaw: float = None, yaw_head_first: bool = False):
        if self.__started == False:
            raise Exception('Drone is not running. '
                            'Please run \'<object_name>.start()\'')
        new_pose = PoseStamped()
        if x is None:
            x = self.__last_command_local_pose.pose.position.x
        if y is None:
            y = self.__last_command_local_pose.pose.position.y
        if z is None:
            z = self.__last_command_local_pose.pose.position.z
        new_pose.pose.position.x = x + self.__start_xyz[0]
        new_pose.pose.position.y = y + self.__start_xyz[1]
        new_pose.pose.position.z = z + self.__start_xyz[2]
        if yaw is None:
            new_pose.pose.orientation = self.__last_command_local_pose.pose.orientation
        else:
            q = quaternion_from_euler(0, 0, yaw)
            new_pose.pose.orientation.x = q[0]
            new_pose.pose.orientation.y = q[1]
            new_pose.pose.orientation.z = q[2]
            new_pose.pose.orientation.w = q[3]
        self.__last_command_local_pose = new_pose
        self.__yaw_head_first = yaw_head_first
        self.__type_of_move = 'LOCAL_POSE'

    def point_is_reached(self):
        if self.is_shutdown() or self.__started == False or self.__type_of_move != 'LOCAL_POSE':
            return True
        delta_x = (self.__last_command_local_pose.pose.position.x
                   - self.__current_local_pose.pose.position.x)
        delta_y = (self.__last_command_local_pose.pose.position.y
                   - self.__current_local_pose.pose.position.y)
        delta_z = (self.__last_command_local_pose.pose.position.z
                   - self.__current_local_pose.pose.position.z)
        distance = sqrt((delta_x)**2 + (delta_y)**2 + (delta_z)**2)
        if distance < self.__allowable_error:
            return True
        else:
            return False

    # GLOBAL_POSE methods
    # alt_type: 'START', 'SEA' or 'ELLIPSOID'
    def get_global_pose(self, alt_type: str = 'START'):
        if alt_type not in ('START', 'SEA', 'ELLIPSOID'):
            raise ValueError(f'alt_type: \'{alt_type}\'. '
                             'Correct values: \'START\', \'SEA\', \'ELLIPSOID\'')
        latitude = self.__current_global_pose.latitude
        longitude = self.__current_global_pose.longitude
        altitude = self.__current_global_pose.altitude
        if alt_type == 'START':
            if self.__started == False:
                altitude = 0
            else:
                altitude -= self.__start_alt
        elif alt_type == 'SEA':
            altitude -= self.__egm96.height(latitude, longitude)
        elif alt_type == 'ELLIPSOID':
            pass
        return latitude, longitude, altitude

    def set_global_pose(self, latitude: float = None, longitude: float = None, altitude: float = None,
                        yaw: float = None, yaw_head_first: bool = False, alt_type: str = 'START'):
        if self.__started == False:
            raise Exception('Drone is not running. '
                            'Please run \'<object_name>.start()\'')
        if alt_type not in ('START', 'SEA', 'ELLIPSOID'):
            raise ValueError(f'alt_type: \'{alt_type}\'. '
                             'Correct values: \'START\', \'SEA\', \'ELLIPSOID\'')
        new_pose = GeoPoseStamped()
        if latitude is None:
            latitude = self.__last_command_global_pose.pose.position.latitude
        if longitude is None:
            longitude = self.__last_command_global_pose.pose.position.longitude
        if altitude is None:
            altitude = self.__last_command_global_pose.pose.position.altitude
            alt_type = 'SEA'
        if alt_type == 'START':
            altitude += self.__start_alt
            altitude -= self.__egm96.height(latitude, longitude)
        elif alt_type == 'SEA':
            pass
        elif alt_type == 'ELLIPSOID':
            altitude -= self.__egm96.height(latitude, longitude)
        new_pose.pose.position.latitude = latitude
        new_pose.pose.position.longitude = longitude
        new_pose.pose.position.altitude = altitude
        if yaw is None:
            new_pose.pose.orientation = self.__last_command_global_pose.pose.orientation
        else:
            q = quaternion_from_euler(0, 0, yaw)
            new_pose.pose.orientation.x = q[0]
            new_pose.pose.orientation.y = q[1]
            new_pose.pose.orientation.z = q[2]
            new_pose.pose.orientation.w = q[3]
        self.__yaw_head_first = yaw_head_first
        self.__last_command_global_pose = new_pose
        self.__type_of_move = 'GLOBAL_POSE'

    # VELOCITY velocity methods
    def set_velocity(self, x: float = None, y: float = None, z: float = None, yaw: float = None):
        if self.__started == False:
            raise Exception('Drone is not running. '
                            'Please run \'<object_name>.start()\'')
        new_vel = Twist()
        if x is None:
            x = self.__last_command_vel.linear.x
        if y is None:
            y = self.__last_command_vel.linear.y
        if z is None:
            z = self.__last_command_vel.linear.z
        if yaw is None:
            yaw = self.__last_command_vel.angular.z
        new_vel.linear.x = x
        new_vel.linear.y = y
        new_vel.linear.z = z
        new_vel.angular.z = yaw
        self.__last_command_vel = new_vel
        self.__type_of_move = 'VELOCITY'


class Telemerty_api:
    def __init__(self):
        # GENERAL attributes
        self.__state = State()

        rospy.init_node('drone_offb', anonymous=True)
        # get state, position, set position, velocity, mode and arm
        self.__state_sub = rospy.Subscriber('mavros/state', State,
                                            self.__state_cb, queue_size=10)

    def __state_cb(self, state):
        self.__state = state

    @property
    def state(self):
        return self.__state


class Camera_api:
    def __init__():
        pass

    @staticmethod
    def __rvec_to_euler_angles(rvec):
        rmat = Rodrigues(rvec)[0]
        sy = sqrt(rmat[0, 0] * rmat[0, 0] + rmat[1, 0] * rmat[1, 0])
        singular = sy < 1e-6

        if not singular:
            roll = atan2(-rmat[2, 0], sy)
            pitch = atan2(rmat[2, 1], rmat[2, 2])
            yaw = atan2(rmat[1, 0], rmat[0, 0])
        else:
            roll = atan2(-rmat[2, 0], sy)
            pitch = atan2(-rmat[1, 2], rmat[1, 1])
            yaw = 0

        if pitch > 0:
            pitch = -pitch + pi
        else:
            pitch = -pitch - pi
        return [-roll, pitch, -yaw]

    @classmethod
    def marker_cam_pose(cls, rvec, tvec):
        # ROLL PITCH YAW OF MARKER
        roll, pitch, yaw = cls.__rvec_to_euler_angles(rvec)
        # X Y Z RELATIVE TO CAMERA
        x = tvec[2]
        y = -tvec[0]
        z = -tvec[1]
        return x, y, z, roll, pitch, yaw

    @classmethod
    def marker_drone_pose(cls, rvec, tvec, camera_pose=(0, 0, 0, 0, 0, 0)):
        """CAMERA POSE: 
        pose[0] : x
        pose[1] : y
        pose[2] : z
        pose[3] : roll
        pose[4] : pitch
        pose[5] : yaw"""
        # http://mathhelpplanet.com/static.php?p=pryeobrazovaniya-pryamougolnyh-koordinat
        x, y, z, roll, pitch, yaw = cls.marker_cam_pose(rvec, tvec)
        # X Y Z RELATIVE TO DRONE
        # yaw - Oxy
        x_new = x*cos(camera_pose[5]) - y*sin(camera_pose[5])
        y = x*sin(camera_pose[5]) + y*cos(camera_pose[5])
        x = x_new
        # pitch - Oxz
        x_new = x*cos(-camera_pose[4]) - z*sin(-camera_pose[4])
        z = x*sin(-camera_pose[4]) + z*cos(-camera_pose[4])
        x = x_new
        # roll - Oyz
        y_new = y*cos(camera_pose[3]) - z*sin(camera_pose[3])
        z = y*sin(camera_pose[3]) + z*cos(camera_pose[3])
        y = y_new
        # camera
        x += camera_pose[0]
        y += camera_pose[1]
        z += camera_pose[2]
        return x, y, z, roll, pitch, yaw

    @classmethod
    def marker_local_pose(cls, rvec, tvec, drone_pose=(0, 0, 0, 0, 0, 0), camera_pose=(0, 0, 0, 0, 0, 0)):
        """DRONE/CAMERA POSE: 
        pose[0] : x
        pose[1] : y
        pose[2] : z
        pose[3] : roll
        pose[4] : pitch
        pose[5] : yaw"""
        x, y, z, roll, pitch, yaw = cls.marker_drone_pose(rvec, tvec,
                                                          camera_pose)
        # X Y Z RELATIVE TO LOCAL COORD
        # yaw - Oxy
        x_new = x*cos(drone_pose[5]) - y*sin(drone_pose[5])
        y = x*sin(drone_pose[5]) + y*cos(drone_pose[5])
        x = x_new
        # pitch - Oxz
        x_new = x*cos(-drone_pose[4]) - z*sin(-drone_pose[4])
        z = x*sin(-drone_pose[4]) + z*cos(-drone_pose[4])
        x = x_new
        # roll - Oyz
        y_new = y*cos(drone_pose[3]) - z*sin(drone_pose[3])
        z = y*sin(drone_pose[3]) + z*cos(drone_pose[3])
        y = y_new
        # drone
        x += drone_pose[0]
        y += drone_pose[1]
        z += drone_pose[2]
        return x, y, z, roll, pitch, yaw
