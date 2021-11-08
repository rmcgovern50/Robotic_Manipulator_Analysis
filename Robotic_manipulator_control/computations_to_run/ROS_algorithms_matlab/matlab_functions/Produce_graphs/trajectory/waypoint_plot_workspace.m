clear
clc

%% setup robot to use
addpath(genpath(strcat(pwd,'\Dependencies')))
robot = loadrobot('universalUR5','dataFormat','column','Gravity', [0,0,-9.81]);%createRigidBodyTree;
eeOffset = 0.01;
eeBody = robotics.RigidBody('end_effector');
setFixedTransform(eeBody.Joint, trvec2tform([eeOffset 0 0]));
addBody(robot, eeBody, 'tool0');
%axes = show(robot);
%axes.CameraPositionMode = 'auto';

numJoints = numel(homeConfiguration(robot));

tspan = 0:0.01:0.5;
q0 = zeros(numJoints, 1);
q0(2) = pi/4;
qd0 = zeros(numJoints, 1); 
initialState = [q0; qd0];

targetJp = [pi/2; pi/3; pi/6; pi/2; pi/3; pi/6];
targetJv = zeros(numJoints,1);
targetJa = zeros(numJoints, 1);

%show(robot, targetJp)

ComputedTorqueMotion = jointSpaceMotionModel("RigidBodyTree", robot, "MotionType", "ComputedTorqueControl");
updateErrorDynamicsFromStep(ComputedTorqueMotion, 0.2, 0.1)
qDesComputedTorque = [targetJp; targetJv; targetJa]; 

[tComputedTorque, yComputedTorque] = ode45(@(t,y)derivative(ComputedTorqueMotion,y,qDesComputedTorque), tspan, initialState);

exampleHelperRigidBodyTreeAnimation(robot, yComputedTorque, 1);
