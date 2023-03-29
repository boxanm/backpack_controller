import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Path

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sklearn.neighbors import NearestNeighbors
import numpy as np
import os
import time
from threading import Lock


class BackpackController(Node):
    frequency = 1500
    base_length = 1.0
    base_duration = 0.1
    dist_rate_dict = {5.0: 1.0,
                      4.0: 0.8,
                      3.0: 0.6,
                      2.0: 0.5,
                      1.0: 0.4,
                      0.5: 0.2,
                      0.0: 0.1}

    def __init__(self):
        super().__init__('norlab_sound_indicators')
        self.publisher_ = self.create_publisher(String, 'distance_to_path', 10)

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.path_subscriber_ = self.create_subscription(Path, 'planned_trajectory_test',
                                                         self.path_callback, qos_profile)

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
            'target_frame', 'base_link').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        beeper_period = self.base_length
        distance_update_period = 0.1  # seconds
        self.beeper_timer = self.create_timer(beeper_period, self.beeper_callback)
        self.distance_update_timer = self.create_timer(distance_update_period, self.timer_callback)
        self.distance_to_traj = None
        self.last_pos = None
        self.knn = NearestNeighbors(n_neighbors=1)
        self.ratio = None
        self.path = None
        self.ratio_mutex = Lock()
        self.get_logger().info("Waiting for path message")

    def play_sound(self, sound_dur, pause_dur, count):
        if sound_dur * count == self.base_length:
            os.system(f'play -n synth {self.base_length} sin {self.frequency} >/dev/null 2>&1')
        elif pause_dur * count == self.base_length:
            time.sleep(self.base_length)
        else:
            for i in range(count):
                os.system(f'play -n synth {sound_dur} sin {self.frequency} >/dev/null 2>&1')
                time.sleep(pause_dur)

    def beeper_callback(self):
        if self.path is None or self.ratio is None:
            return

        self.ratio_mutex.acquire()
        sound_dur_total = self.ratio * self.base_length
        sound_count = sound_dur_total / self.base_duration
        sound_dur_indiv = self.base_duration

        pause_dur_total = self.base_length - sound_dur_total
        if sound_count == 0.0:
            pause_count = self.base_length
        else:
            pause_count = sound_count
        pause_dur_indiv = pause_dur_total / pause_count

        print(
            f"Playing sound for distance {self.distance_to_traj} with ratio {self.ratio}: sound_dur_ind {sound_dur_indiv}, pause_dur_ind {pause_dur_indiv}")
        print(
            f"Total sound dur {sound_dur_total} with count {sound_count} total pause dur {pause_dur_total} with count {pause_count}")

        self.ratio_mutex.release()
        self.play_sound(sound_dur_indiv, pause_dur_indiv, int(sound_count))

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
            self.ratio_mutex.acquire()
            self.ratio = None
            self.ratio_mutex.release()
            return

        self.last_pos = tf.transform.translation
        self.distance_to_traj = self.compute_dist_to_traj(
            np.array([self.last_pos.x, self.last_pos.y]))
        self.ratio_mutex.acquire()
        self.ratio = 0.1
        for i in range(len(self.dist_rate_dict) - 1):
            if self.distance_to_traj >= list(self.dist_rate_dict.keys())[i]:
                self.ratio = list(self.dist_rate_dict.values())[i]
                break
        self.ratio_mutex.release()
        msg = String()
        msg.data = f'Distance to target: {self.distance_to_traj}'
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)

    def path_callback(self, path):
        if not path.poses:
            self.get_logger().warn("Received empty path, skipping")
            return
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
    norlab_sound_indicators = BackpackController()

    rclpy.spin(norlab_sound_indicators)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    norlab_sound_indicators.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
