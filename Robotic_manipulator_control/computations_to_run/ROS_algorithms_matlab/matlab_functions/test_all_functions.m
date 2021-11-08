%%  robot_type = return_robot_object(label)

robot = 'universalUR5';
UR5 = return_robot_object(robot);

%% configuration = return_home_configuration(label)

robot = 'universalUR5';
home_configuration = return_home_configuration(robot);

%% configuration = return_random_configuration(label)

robot = 'universalUR5';
home_configuration = return_random_configuration(robot);

%% isAdmissible = check_if_admissible(bounds,x1, x2)

x1 = 0.25;
x2 = 3;
robot_label = 'universalUR5';

result = check_if_admissible(robot_label, x1, x2);

%% [A,D] = get_input_limits(configuration, joint_velocity)
configuration = [0,0,0,0,0,0];
joint_velocity = [6, -6, 6, -6, -6, 6]; % [0.05, -0.1, 2, -2, -0.6, 6];
clc
[A, D] = get_input_limits(configuration, joint_velocity);
A