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

from interbotix_xs_modules.xs_robot.locobot import InterbotixLocobotXS

"""
This script commands the base to move to an arbitrary point on a map.
Note that this script assumes you already have a map built

To get started, open a terminal and run the command:

    ros2 launch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_wx250s
        use_nav:=true 

Then change to this directory and run the command:

    python3 move_base.py

or run the command:

    ros2 run interbotix_xslocobot_control move_base.py
"""

from geometry_msgs.msg import Pose



def main():
    locobot = InterbotixLocobotXS(
        robot_model='locobot_wx250s',
        robot_name='locobot',
        arm_model='mobile_wx250s',
        use_base = True,
        use_nav = True,
    )
    # import pdb; pdb.set_trace()
    # locobot.base.command_pose_xyyaw(x=0.1, y=0., yaw=0, blocking=True)
    # locobot.base.command_position_xyyaw(x=0.01, y=0., yaw=0, blocking=True)
    
    # Create the goal position
    goal_position = Pose()
    goal_position.position.x = 0.02  # Move 2 cm forward along the x-axis
    goal_position.position.y = 0.0   # No lateral movement
    goal_position.orientation.z = 0.0  # No rotation, keep the orientation as is

    # Command the robot to move forward 2 cm
    locobot.base.command_position(goal_position=goal_position, blocking=False)


    locobot.shutdown()


if __name__ == '__main__':
    main()
