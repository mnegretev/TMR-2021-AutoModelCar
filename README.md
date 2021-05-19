# TMR 2021 - TeamCIIAM

# Para ejecutar conducción autónoma sin obstáculos

roslaunch self_autonomous self_autonomous.launch

rostopic b -r 1 AutoModelMini/manual_control/speed std_msgs/Int16 '{data: -600}'
