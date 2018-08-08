#!/bin/bash

rosparam delete /virtual_bumper
rosparam load virtual_bumper.yaml /virtual_bumper
rostopic pub -1 /virtual_bumper/reload_parameters std_msgs/Empty '{}'


