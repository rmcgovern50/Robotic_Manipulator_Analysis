function robot_type = return_robot_object(label)
robot = loadrobot(label,'Gravity', [0,0,-9.81]);
%ur3 = loadrobot('universalUR3','Gravity', [0,0,-9.81]);
%ur10 = loadrobot('universalUR10','Gravity', [0,0,-9.81]);
robot.DataFormat = "row";

robot_type = robot
end

