import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Path

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sklearn.neighbors import NearestNeighbors
import numpy as np
from preferredsoundplayer import playsound, stopsound

class BackpackController(Node):

    dist_rate_dict = {5.0: 1.0,
                      4.0: 0.7,
                      0.5: 0.5,
                      0.3: 0.3,
                      0.2: 0.1,
                      0.0: 0.0}

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'distance_to_path', 10)
        self.path_subscriber_ = self.create_subscription(Path, 'planned_trajectory',
                                                         self.path_callback, 10)

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
            'target_frame', 'base_link').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        beeper_period = 0.1
        timer_period = 1.0  # seconds
        self.beeper_timer = self.create_timer(beeper_period, self.beeper_callback)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.distance_to_traj = 0
        self.path = None
        self.last_pos = None
        self.knn = NearestNeighbors(n_neighbors=1)
        self.get_logger().info("Waiting for path message")
        self.sound_ratio = 0.0
        self.new_sound_ratio = 0.0
        self.new_ratio_ready = False
        self.sound_ctr = 0.0
        self.sound = None

    def beeper_callback(self):
        if self.new_ratio_ready and self.sound_ctr == 0.0:
            print(f"new sound ratio ready: {self.sound_ratio}, new: {self.new_sound_ratio}")
            self.sound_ratio = self.new_sound_ratio
        if self.sound is not None:
            stopsound(self.sound)

        if self.sound_ctr < self.sound_ratio:
            self.sound = playsound('../resource/beep.wav')
            print(f"playing sound {self.sound_ctr}")
        else:
            print(f'not playing sound {self.sound_ctr}')
        self.sound_ctr += 0.1
        if self.sound_ctr >= 1.0:
            self.sound_ctr = 0.0

    def timer_callback(self):
        # get latest TF
        if self.path is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                self.target_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {self.target_frame} to map: {ex}')
            return

        self.last_pos = tf.transform.translation
        self.distance_to_traj = self.compute_dist_to_traj(
            np.array([self.last_pos.x, self.last_pos.y]))
        for i in range(len(self.dist_rate_dict) - 1):
            if list(self.dist_rate_dict.keys())[i] > self.distance_to_traj >= \
                    list(self.dist_rate_dict.keys())[i + 1]:
                self.new_sound_ratio = list(self.dist_rate_dict.values())[i + 1]
        if self.new_sound_ratio != self.sound_ratio:
            self.new_ratio_ready = True
        else:
            self.new_ratio_ready = False
        msg = String()
        msg.data = f'Distance to target: {self.distance_to_traj}'
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)

    def path_callback(self, path):
        self.path = path

        poses = self.path.poses
        coords_vect = np.vstack(
            [[pose.pose.position.x for pose in poses], [pose.pose.position.y for pose in poses]]).T
        self.knn.fit(coords_vect)

    def compute_dist_to_traj(self, curr_pos):
        # knn implementation
        distance_mat, neighbours_mat = self.knn.kneighbors(curr_pos.reshape(1, -1))
        print(distance_mat)
        return distance_mat[0][0]


def main(args=None):
    rclpy.init(args=args)
    backpack_controller = BackpackController()

    rclpy.spin(backpack_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    backpack_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
