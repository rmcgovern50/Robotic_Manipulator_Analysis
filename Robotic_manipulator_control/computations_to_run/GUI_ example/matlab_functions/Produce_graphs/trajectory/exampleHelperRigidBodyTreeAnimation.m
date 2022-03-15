function exampleHelperRigidBodyTreeAnimation(robot, stateData, dispInterval, targetPos)
%displayRobot Animate the robot motion
%   Given a ROBOT, the array of states defined in STATEDATA, visualize the
%   robot position by animating it inside a for-loop. The frame display
%   interval set by DISPINTERVAL controls the number of frames between each
%   subsequent animation (increase to speed up animation), while the
%   optional value TARGETPOS adds a marker at the specified target
%   position.
        
    % Define number of moving robot joints
    numInputs = numel(homeConfiguration(robot));

    % Create figure and hold it
    figure
    set(gcf,'Visible','on');
    show(robot, stateData(1, 1:numInputs)');
    hold on
    
    if nargin > 3
        plot3(targetPos(1), targetPos(2), targetPos(3), 'x', 'MarkerSize', 10');
    end
    
    % Loop through values at specified interval and update figure
    for j = 1:dispInterval:length(stateData)
        % Display manipulator model
        qDisp = stateData(j, 1:numInputs)';
        show(robot, qDisp, 'Frames', 'off', 'PreservePlot', false);
        title(sprintf('Frame = %d of %d', j, length(stateData)));

        % Update figure
        drawnow
    end
end