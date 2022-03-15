function torques = Calc_torque_required(robot_label, x1, x2, qx1_eval, dqx1_eval, ddqx1_eval, u_val)

%CALC_TORQUE_REQUIRED Summary of this function goes here
%   Detailed explanation goes here

robot_model = loadrobot(robot_label,'Gravity', [0,0,-9.81]);
robot_model.DataFormat = "row";

%% The torque limits for the UR5

tau_max = [150   150  150  28  28  28];
tau_min = [-150 -150 -150 -28 -28 -28];

%% work out the parameters
qx1_evaluated = cell2mat(qx1_eval); %B = [A{:}]
dqx1_evaluated = cell2mat(dqx1_eval);
ddqx1_evaluated = cell2mat(ddqx1_eval);
dqdt_evaluated = dqx1_evaluated*x2;


%% Path dynamics parameters
Ms = massMatrix(robot_model, single(qx1_evaluated))*transpose(dqx1_evaluated);

Cs = massMatrix(robot_model, single(qx1_evaluated))*transpose(ddqx1_evaluated)*x2^2 + ...
     transpose(velocityProduct(robot_model, single(qx1_evaluated), single(dqdt_evaluated)));
 
gs= transpose(gravityTorque(robot_model, single(qx1_evaluated)));

tau = (Ms)*u_val + Cs + gs;
torques = tau;
end

