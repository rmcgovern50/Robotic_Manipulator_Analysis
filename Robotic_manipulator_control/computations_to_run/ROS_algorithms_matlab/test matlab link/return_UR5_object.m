function UR5 = return_UR5_object()
robot = loadrobot('universalUR5','Gravity', [0,0,-9.81]);
%ur3 = loadrobot('universalUR3','Gravity', [0,0,-9.81]);
%ur10 = loadrobot('universalUR10','Gravity', [0,0,-9.81]);
robot.DataFormat = "row";

UR5 = robot
end

