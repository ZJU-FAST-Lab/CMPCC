roslaunch so3_quadrotor_simulator simulator.launch & sleep 1;
roslaunch environment map_generator.launch & sleep 1;
roslaunch local_sensing_node sensing.launch & sleep 1;
roslaunch plan_env planning.launch & sleep 1;
rosrun cmpcc simulation_node
wait

