roslaunch so3_quadrotor_simulator visualization.launch & sleep 2;
roslaunch so3_quadrotor_simulator simulator.launch & sleep 1;
roslaunch environment map_generator.launch & sleep 1;
python key2joy.py & sleep 1;
rosrun cmpcc simulation_node
wait

