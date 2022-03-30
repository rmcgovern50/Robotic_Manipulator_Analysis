function isAdmissible = check_if_admissible(robot_label, x1, x2, qx1_eval, dqx1_eval, ddqx1_eval)
% CHECK_IF_ADMISSIBLE Summary of this function goes here
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

%% now work out the acceleration limits

ai = (tau_max - Cs - gs)./Ms;
di = (tau_min - Cs - gs)./Ms;
   
val = [];
A = [];
D = [];
for i = 1:length(Ms)
    if Ms(i) > 0
        A(end+1) = ai(i);
        D(end+1) = di(i);
    else
        A(end+1) = di(i);
        D(end+1) = ai(i);
    end
end
    
%% return true if admissible false if not

if min(A) > max(D)
    isAdmissible = true; 
else
    isAdmissible = false;
end

end

