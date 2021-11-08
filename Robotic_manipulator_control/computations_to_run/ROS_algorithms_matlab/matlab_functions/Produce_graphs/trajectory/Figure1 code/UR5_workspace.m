clear
clc

%% setup robot to use
addpath(genpath(strcat(pwd,'\Dependencies')))
robot = loadrobot('universalUR5','Gravity', [0,0,-9.81]);%createRigidBodyTree;
eeOffset = 0.01;
eeBody = robotics.RigidBody('end_effector');
setFixedTransform(eeBody.Joint, trvec2tform([eeOffset 0 0]));
addBody(robot, eeBody, 'tool0');
axes = show(robot);
axes.CameraPositionMode = 'auto';


%% set waypoints
wayPoints = create_circular_arc_waypoints();%[0.6 -0.4 0; 0.5 0 0.7; 0.4 0.4 0.4; 0.1 0.4 0]; % Alternate set of wayPoints
%wayPointVels = create_circular_arc_waypoints()/100;%[0 0 0;0 0.1 0;0 0 0; 0 0 0];
%exampleHelperPlotWaypoints(wayPoints);

%% Create a smooth trajectory from the waypoints
%numTotalPoints = 50;
%waypointTime = 4;
%trajType = 'cubic'; % or 'trapezoidal'

%wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
%trajTimes = linspace(0,wpTimes(end),numTotalPoints);

trajectory = transpose(wayPoints);%cubicpolytraj(wayPoints',wpTimes,trajTimes, 'VelocityBoundaryCondition',wayPointVels');


%% Plot trajectory spline and waypoints
hold on
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);

%%

ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0.1 1 1 1];
initialguess = robot.homeConfiguration;

%% % Call inverse kinematics solver for every end-effector position using the
% previous configuration as initial guess

for idx = 1:size(trajectory,2)
    tform = trvec2tform(trajectory(:,idx)');
    configSoln(idx,:) = ik('end_effector',tform,weights,initialguess);
    initialguess = configSoln(idx,:);
end

%% animate Visualize robot configurations
title('Robot waypoint tracking visualization')
%axis([-0.7 0.4 -0.35 0.35 0 0.7]);
for idx = 1:size(trajectory,2)
    show(robot,configSoln(idx,:), 'PreservePlot', false,'Frames','off');
    pause(0.1)
end
hold off





