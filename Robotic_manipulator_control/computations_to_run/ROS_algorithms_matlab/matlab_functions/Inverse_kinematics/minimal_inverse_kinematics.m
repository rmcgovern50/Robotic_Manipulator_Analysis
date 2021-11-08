%clear
clc

%% setup robot to use
%addpath(genpath(strcat(pwd,'\Dependencies')))
robot = loadrobot('universalUR5','Gravity', [0,0,-9.81]);%createRigidBodyTree;
eeOffset = 0.0; %set the end effector offset
eeBody = robotics.RigidBody('end_effector'); %create a body to act as the end effector
setFixedTransform(eeBody.Joint, trvec2tform([eeOffset eeOffset eeOffset])); %relate the position of the end effector to body
addBody(robot, eeBody, 'tool0'); %add end effector to the body


%% set waypoints (we need velocities for the profiles)
wayPoints = create_circular_arc_waypoints();%get some waypoints in this case a circular arc
wayPointVels = wayPoints/100; %set some arbitrary velocities

%% Create a smooth trajectory from the waypoints
numTotalPoints = 50; %number of points in totoal
waypointTime = 4; %not sure what this is doing?

trajType = 'cubic'; % or 'trapezoidal'

%next 3 lines allow a trajectory to be formed to the specifications above
wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
trajTimes = linspace(0,wpTimes(end),numTotalPoints);
trajectory = cubicpolytraj(wayPoints',wpTimes,trajTimes, 'VelocityBoundaryCondition',wayPointVels');
%% set up to do inverse kinematics

ik = robotics.InverseKinematics('RigidBodyTree',robot); %set up the inverse kinematics solver
weights = [1 1 1 1 1 1]; %set weights that decribe importance of the various links
initialguess = robot.homeConfiguration; %the solver needs an initial guess

%% % Call inverse kinematics solver for every end-effector position using the
% previous configuration as initial guess


q = []; %matrix to be populated with each row being a different configuration on the path

%loop does the inverse kinematics
for idx = 1:size(trajectory,2)
    tform = trvec2tform(trajectory(:,idx)');
    configSoln(idx,:) = ik('end_effector', tform,weights, initialguess);
    initialguess = configSoln(idx,:);
    
    %read the joint positions each time and save them in q
    [q1, q2, q3, q4, q5, q6] = initialguess.JointPosition;
    q(end+1,:) = [q1, q2, q3, q4, q5, q6];
end


q


