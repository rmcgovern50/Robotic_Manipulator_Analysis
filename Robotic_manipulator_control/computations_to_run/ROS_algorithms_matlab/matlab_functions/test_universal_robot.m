ur5 = loadrobot('universalUR5','Gravity', [0,0,-9.81]);
%ur3 = loadrobot('universalUR3','Gravity', [0,0,-9.81]);
%ur10 = loadrobot('universalUR10','Gravity', [0,0,-9.81]);
ur5.DataFormat = "row";

%define the startg and end of a path
q = homeConfiguration(ur5);
q_target = [pi/2 pi/2 pi/3 2*pi/3 -pi/2 -pi/3];

%define the path as a symbolic equation
x1 = sym('x1');
qx1 = q + x1*(q_target - q);
dqx1 = diff(qx1);
ddqx1 = diff(dqx1);

% the actual derivatives symbolically
x2 = sym('x2');
u = sym('u');
dqdt = dqx1*x2;
ddqdt2 = dqx1*u + ddqx1*x2^2;

%evaluate the path
x1_val = 0.5;
qx1_evaluated = subs(qx1, x1, x1_val)
dqx1_evaluated = subs(dqx1, x1, x1_val);
ddqx1_evaluated = subs(ddqx1, x1, x1_val);

%evalate the time derivatives
x2_val = 0.000000000001;
dqdt_evaluated = subs(dqdt, [x1, x2], [x1_val, x2_val]);
ddqdt2_evaluated = subs(ddqdt2, [x1, x2], [x1_val, x2_val])




%how to get lagrangian dynamics values at a particular qx_1
%M_L = massMatrix(ur5, qx1_evaluated) %M 
%C = velocityProduct(ur5,q,qd) %Cdq/dt
%g = gravityTorque(ur5,q)

%Path dynamics parameters
Ms = massMatrix(ur5, single(qx1_evaluated))*transpose(dqx1_evaluated);
Cs = massMatrix(ur5, single(qx1_evaluated))*transpose(ddqx1_evaluated) + ...
     transpose(velocityProduct(ur5, single(qx1_evaluated), single(dqdt_evaluated))* (1/x2_val)); 
gs= transpose(gravityTorque(ur5,single(qx1_evaluated)));

u_val = 0.0
tau = (Ms)*u_val +(Cs)*x2_val^2 + gs


tau2 = inverseDynamics(ur5, single(qx1_evaluated)) 


difference = round(tau - transpose(tau2), 6)
show(ur5);

