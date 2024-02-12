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
        
        fwd_val = 0.15
        rot_val = 0.15
        
        min_dist = 0.3
        max_dist = 6
        
        lim_iz_turn = 134
        lim_iz_fwd = 146
        lim_dr_fwd = 213
        lim_dr_turn = 225
        
        left_area = ranges[lim_iz_turn:lim_iz_fwd]
        
        fwd_area = ranges[lim_iz_fwd:lim_dr_fwd]
        
        right_area = ranges[lim_dr_fwd:lim_dr_turn]
        
        vx = 0.
        wz = 0.
        for degree in fwd_area:
                if min_dist < degree < max_dist:
                        vx = fwd_val
        for degree in left_area:
                if min_dist < degree < max_dist:
                        wz = rot_val
        for degree in right_area:
                if min_dist < degree < max_dist:
                        wz = -rot_val
        #print(fwd_area)
        #print(f"Angle min: {angle_min}\nAngle max: {angle_max}\nAngle Increment: {angle_increment}")
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
