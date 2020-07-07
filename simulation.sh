roslaunch so3_quadrotor_simulator simulator.launch & sleep 1;
roslaunch environment map_generator.launch & sleep 1;
rosrun cmpcc simulation_node
wait

