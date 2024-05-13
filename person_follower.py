# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PersonFollower(Node):

    def __init__(self):
        super().__init__('person_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, 
                     history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile=qos_policy)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, input_msg):
        angle_min = input_msg.angle_min
        angle_max = input_msg.angle_max
        angle_increment = input_msg.angle_increment
        ranges = input_msg.ranges
        
        
        min_vel = 0.2
        fwd_val = 0.8
        rot_val = 0.6
        
        min_dist = 0.6
        max_dist = 1.6
        
        turn_slope = 28
        
        center = 45
        det_area = 40
        lim_iz = center - det_area
        lim_dr = center + det_area
        
        detection_area = ranges[lim_iz:lim_dr]
        area_rotation = ranges[0:360]
        
        vx = 0.
        wz = 0.
        
        idx = 0
        for dist_inDegree in detection_area:
                if min_dist < dist_inDegree < max_dist:
                        vx = min_vel + dist_inDegree/max_dist * fwd_val/2
                        if(idx < turn_slope):
                                wz = -rot_val
                        if(idx > (lim_dr - lim_iz - turn_slope)):
                                wz = rot_val
                idx += 1
        idx = 0

        for dist_inDegree in ranges:
                if 0 < dist_inDegree < min_dist:
                        if(100 < idx < 540):
                                wz = rot_val * 2
                        elif(545 < idx < 980):
                                wz = -rot_val * 2
                idx += 1
        
        output_msg = Twist()
        output_msg.linear.x = vx
        output_msg.angular.z = wz
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
