#!/usr/bin/env python3

# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

from interbotix_xs_modules.xs_robot.locobot import InterbotixLocobotXS

"""
This script makes the end-effector perform pick, pour, and place tasks.

To get started, open a terminal and run the command:

    ros2 launch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_wx250s

Then change to this directory and run the command:

    python3 bartender.py

...or run the command:

    ros2 run interbotix_xslocobot_control bartender.py
"""


def main():
    locobot = InterbotixLocobotXS(
        robot_model='locobot_wx250s',
        robot_name='locobot',
        arm_model='mobile_wx250s'
    )

    locobot.arm.set_ee_pose_components(x=0.3, z=0.2)
    locobot.arm.set_single_joint_position('waist', math.pi/4.0)
    locobot.gripper.release()
    locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.25)
    locobot.gripper.grasp()
    locobot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.25)
    locobot.arm.set_single_joint_position('waist', -math.pi/4.0)
    locobot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    locobot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    locobot.arm.set_single_joint_position('waist', math.pi/4.0)
    locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.25)
    locobot.gripper.release()
    locobot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.25)
    locobot.arm.go_to_home_pose()
    locobot.arm.go_to_sleep_pose()

    locobot.shutdown()


if __name__ == '__main__':
    main()
