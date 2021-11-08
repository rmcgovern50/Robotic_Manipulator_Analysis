function configuration = return_home_configuration(label)
%RETURN_HOME_CONFIGURATION Summary of this function goes here
%   Detailed explanation goes here

robot_model = loadrobot(label,'Gravity', [0,0,-9.81]);
robot_model.DataFormat = "row";
configuration = homeConfiguration(robot_model)


end

