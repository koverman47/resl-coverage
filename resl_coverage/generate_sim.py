#!/usr/bin/env python

'''
' No Dynamics
'''

import sys

f = open("./launch/resl_simulation_spawn.launch", "w")
l = open("./launch/resl_simulation_liftoff.launch", "w")
f.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
l.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
f.write("<launch>\n")
l.write("<launch>\n")
f.write("\t<arg name=\"launch_unity\" default=\"true\"/>\n")
#f.write("\t<arg name=\"simulator_param_file\" default=\"$(find arl_unity_ros)/config/overpasscity.yaml\" />\n\n")
f.write("\t<arg name=\"simulator_param_file\" default=\"$(find arl_unity_ros)/config/floodedgrounds.yaml\" />\n\n")
f.write("\t<include file=\"$(find arl_unity_ros)/launch/simulator.launch\">\n")
f.write("\t\t<arg name=\"launch_unity\" value=\"$(arg launch_unity)\"/>\n")
f.write("\t\t<arg name=\"param_file\" value=\"$(arg simulator_param_file)\"/>\n")
f.write("\t</include>\n\n")
f.write("\t<arg name=\"quadrotor_description\" default=\"$(find arl_unity_ros_abstract)/config/quadrotor.yaml\"/>\n\n")
f.write("\t<arg name=\"husky_description\" default=\"$(find arl_unity_ros_abstract)/config/husky.yaml\"/>\n\n")

targets = int(sys.argv[1])
trackers = int(sys.argv[2])

# Trackers
x = str(0)
y = str(3)
counter = 0
for i in range(trackers):
    s = str(i)
    if i % 2 == 0:
        x = str(2 * counter)
        counter += 1
    else:
        x = str(-2 * counter)
    f.write("\t<include file=\"$(find resl_coverage)/launch/abstract_quadrotor.launch\">\n")
    f.write("\t\t<arg name=\"name\" value=\"tracker"+s+"\"/>\n")
    f.write("\t\t<arg name=\"description\" value=\"$(arg quadrotor_description)\" />\n")
    f.write("\t\t<arg name=\"x\" value=\""+x+"\"/>\n")
    f.write("\t\t<arg name=\"y\" value=\""+y+"\"/>\n")
    f.write("\t\t<arg name=\"z\" value=\"1\"/>\n")
    f.write("\t\t<arg name=\"roll\" value=\"0\"/>\n")
    f.write("\t\t<arg name=\"pitch\" value=\"0\"/>\n")
    f.write("\t\t<arg name=\"yaw\" value=\"0\"/>\n")
    f.write("\t</include>\n")
    l.write("\t<include file=\"$(find resl_coverage)/launch/abstract_tracker_mission.launch\">\n")
    l.write("\t\t<arg name=\"name\" value=\"tracker"+s+"\"/>\n")
    l.write("\t\t<arg name=\"control_args\" value=\""+x+" "+y+" 5\"/>\n")
    l.write("\t\t<arg name=\"track_args\" value=\""+str(targets)+" "+str(trackers)+" "+x+" "+y+"\"/>\n")
    l.write("\t</include>\n")


# Targets
x = str(0)
y = str(0)
counter = 0
for i in range(targets):
    s = str(i)
    if i % 2 == 0:
        x = str(2 * counter)
        counter += 1
    else:
        x = str(-2 * counter)
    f.write("\t<include file=\"$(find resl_coverage)/launch/abstract_husky.launch\">\n")
    f.write("\t\t<arg name=\"name\" value=\"target"+s+"\"/>\n")
    f.write("\t\t<arg name=\"description\" value=\"$(arg husky_description)\" />\n")
    f.write("\t\t<arg name=\"x\" value=\""+x+"\"/>\n")
    f.write("\t\t<arg name=\"y\" value=\""+y+"\"/>\n")
    f.write("\t\t<arg name=\"z\" value=\"0.4\"/>\n")
    f.write("\t\t<arg name=\"roll\" value=\"0\"/>\n")
    f.write("\t\t<arg name=\"pitch\" value=\"0\"/>\n")
    f.write("\t\t<arg name=\"yaw\" value=\"0\"/>\n")
    f.write("\t</include>\n")
    l.write("\t<include file=\"$(find resl_coverage)/launch/abstract_target_mission.launch\">\n")
    l.write("\t\t<arg name=\"name\" value=\"target"+s+"\"/>\n")
    l.write("\t\t<arg name=\"control_args\" value=\""+x+" "+y+" 0\"/>\n")
    l.write("\t</include>\n")

l.write("\t<group ns=\"base_station_group\">\n")
l.write("\t\t<node pkg=\"resl_coverage\" type=\"base_station.py\" name=\"base_station\" args=\""+str(targets)+" "+str(trackers)+"\" output=\"screen\"/>\n")
l.write("\t</group>\n")

f.write("</launch>\n")
l.write("</launch>\n")
f.close()
l.close()
