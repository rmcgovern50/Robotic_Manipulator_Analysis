function joint_configuration_list = IK_solver(robot_label, wayPoints)
%IK_SOLVER Summary of this function goes here
% this function takes a string robot_label and a list of waypoints
% the return is the inverse kinematic solution for each waypoint

%% setup robot to use
%addpath(genpath(strcat(pwd,'\Dependencies')))
robot = loadrobot(robot_label,'Gravity', [0,0,-9.81]);%createRigidBodyTree;

%% Create a smooth trajectory from the waypoints
wayPoints = cell2mat(wayPoints)
trajectory = transpose(wayPoints);
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
    configSoln(idx,:) = ik('tool0', tform,weights, initialguess);
    initialguess = configSoln(idx,:);
    %read the joint positions each time and save them in q
    [q1, q2, q3, q4, q5, q6] = initialguess.JointPosition;
    q(end+1,:) = [q1, q2, q3, q4, q5, q6];
end

joint_configuration_list = q;
end

