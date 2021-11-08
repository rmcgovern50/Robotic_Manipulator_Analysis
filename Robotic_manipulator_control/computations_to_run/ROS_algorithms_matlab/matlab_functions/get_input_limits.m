function [A,D] = get_input_limits(x1, x2, qx1_evaluated, dqx1_evaluated, ddqx1_evaluated, dqdt_evaluated)
%GET_INPUT_LIMITS Summary of this function goes here
%   Detailed explanation goes here
label = "universalUR5";
robot = loadrobot(label,'Gravity', [0,0,-9.81]);
%ur3 = loadrobot('universalUR3','Gravity', [0,0,-9.81]);
%ur10 = loadrobot('universalUR10','Gravity', [0,0,-9.81]);
robot.DataFormat = "row";



%% set torque limits for each joint
%define torque limits of the particular robot

%believed order
%tau = [Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist3]
%valued list with joint limits in Nm
tau = [150, 150, 150, 28, 28, 28];

%% acquire the values relating to the path pass these from python perhaps as a list and unpack here

% qx1_evaluated
% dqx1_evaluated
% ddqx1_evaluated
% dqdt_evaluated


%% evaluate M(x1) C(x1), g(x1), knowledge of paths needed 
% Ms = massMatrix(ur5, single(qx1_evaluated))*transpose(dqx1_evaluated);
% 
% Cs = massMatrix(ur5, single(qx1_evaluated))*transpose(ddqx1_evaluated) + ...
%      transpose(velocityProduct(ur5, single(qx1_evaluated), single(dqdt_evaluated))* (1/x2_val)); 
% 
% gs= transpose(gravityTorque(ur5,single(qx1_evaluated)));
% 

%% Calculate the numerator  tau - C - g;

%numerator1 = tau_max - C x2^2 - g;
%numerator2 = tau_min - C x2^2 - g;

%% work out limits to return
% if Ms ~= 0
%     if Ms > 0
%         %%return values
%         A = numerator1/Ms;
%         D = numerator2/Ms;
%     else Ms < 0
%         A = numerator2/Ms;
%         D = numerator1/Ms;
%     end
% else
%     A = -1
%     D = 1
% end

end

