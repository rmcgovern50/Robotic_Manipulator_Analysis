function [path_equation] = path_symbolic(q_start, q_target)

ur5 = loadrobot('universalUR5','Gravity', [0,0,-9.81]);
%ur3 = loadrobot('universalUR3','Gravity', [0,0,-9.81]);
%ur10 = loadrobot('universalUR10','Gravity', [0,0,-9.81]);
ur5.DataFormat = "row";

%define the startg and end of a path
q_start = homeConfiguration(ur5);

%define the path as a symbolic equation
x1 = sym('x1');
qx1 = q_start + x1*(q_target - q_start);
dqx1 = diff(qx1);
ddqx1 = diff(dqx1);

path_equation = qx1
end

