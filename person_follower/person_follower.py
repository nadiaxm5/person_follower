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
        
        fwd_val = 0.2
        rot_val = 0.3
        
        min_dist = 0.6
        max_dist = 1.4
        
        turn_slope = 26
        
        lim_iz = 144
        lim_dr = 215
        
        detection_area = ranges[lim_iz:lim_dr]
        
        vx = 0.
        wz = 0.
        
        idx = 0
        for degree in detection_area:
                if min_dist < degree < max_dist:
                        vx = 0.06 + degree/max_dist * fwd_val/2
                        if(idx < turn_slope):
                                wz = rot_val
                        if(idx > (lim_dr - lim_iz - turn_slope)):
                                wz = -rot_val
                idx += 1
                print(f"vx: {vx}")
        #for degree in fwd_area:
        #        if min_dist < degree < max_dist:
        #                vx = fwd_val
        #for degree in left_area:
        #        if min_dist < degree < max_dist:
        #                wz = rot_val
        #for degree in right_area:
        #        if min_dist < degree < max_dist:
        #                wz = -rot_val
                        
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
