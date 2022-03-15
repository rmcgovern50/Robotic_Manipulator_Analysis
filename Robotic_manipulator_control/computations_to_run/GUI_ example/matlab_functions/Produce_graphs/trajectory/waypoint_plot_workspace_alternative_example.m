robot = loadrobot("abbIrb120T","DataFormat","column","Gravity",[0 0 -9.81]);
numJoints = numel(homeConfiguration(robot));

% Set up simulation parameters
tSpan = 0:0.01:0.5;
q0 = zeros(numJoints,1);
q0(2) = pi/4; % Something off center
qd0 = zeros(numJoints,1);
initialState = [q0; qd0];

% Set up joint control targets
targetJointPosition = [pi/2 pi/3 pi/6 2*pi/3 -pi/2 -pi/3]';
targetJointVelocity = zeros(numJoints,1);
targetJointAcceleration = zeros(numJoints,1);

computedTorqueMotion = jointSpaceMotionModel("RigidBodyTree",robot,"MotionType","ComputedTorqueControl");
updateErrorDynamicsFromStep(computedTorqueMotion,0.2,0.1);

qDesComputedTorque = [targetJointPosition; targetJointVelocity; targetJointAcceleration];

[tComputedTorque,yComputedTorque] = ode45(@(t,y)derivative(computedTorqueMotion,y,qDesComputedTorque),tSpan,initialState);

exampleHelperRigidBodyTreeAnimation(robot,yComputedTorque,1);