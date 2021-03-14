xterm -e  " roslaunch my_robot world.launch" &
sleep 5
xterm -e  " roslaunch my_robot amcl_demo.launch" &
sleep 5
xterm -e  " roslaunch my_robot gmapping.launch" &
sleep 5
xterm -e  " roslaunch my_robot home_service_robot.launch" 
