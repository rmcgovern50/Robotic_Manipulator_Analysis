function configuration = return_random_configuration(label)
%RETURN_RANDOM_CONFIGURATION Summary of this function goes here
%   Detailed explanation goes here
robot_model = loadrobot(label,'Gravity', [0,0,-9.81]);
robot_model.DataFormat = "row";
configuration = randomConfiguration(robot_model)
end

