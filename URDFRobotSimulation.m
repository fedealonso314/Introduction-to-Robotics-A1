% Clean start
clear; clc; close all;

% --------- USER SETTINGS ----------
urdfFile = "URDF_Second.URDF";   
% ----------------------------------
robotFolder = fileparts(which(urdfFile));
addpath(genpath(robotFolder));
setenv('URDF_Second.URDF', robotFolder);

robot = importrobot(urdfFile);
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

% Home configuration
q = homeConfiguration(robot);

% Create figure
figure();
ax = axes;
hold(ax,'on');

% Show robot with meshes
show(robot, q, 'Visuals','on', 'Collisions','off', 'Frames','off', 'PreservePlot', false,'Parent', ax);

% Visualization settings
axis equal
view(135,25)
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on
camlight('headlight')

% Manually set fixed limits (adjust these values based on your robot size)
xlim([-1 1]); 
ylim([-1 1]); 
zlim([0 1]);
q = [0.65 pi/2 0 0]; 
qStart = q;
qMiddle = qStart; 
qGoal  = qMiddle;
qMiddle(1) = 0.65;     
qMiddle(2) = 0;
qMiddle(3) = 0;
qMiddle(4) = 0;
qGoal(1) = 0.65;     
qGoal(2) = -pi/2;
qGoal(3) = 0;
qGoal(4) = 0;
% Animation of the desired trajectory 
nSteps = 100;
[qMatrix,~,~] = trapveltraj([qStart; qGoal]', nSteps);
% --- Configuration variables ---
endEffectorName = 'link4'; % Assuming Link4 is the end-effector
nSteps = 100;
% Define the angular threshold in radians (10 degrees)
ANGLE_THRESHOLD_RAD = deg2rad(5); 
q_joint_sampled_last = qMatrix(1, 2); 


pathHandles = {}; 
pointIndex = 1;

for i = 1:nSteps

    currentQ = qMatrix(:,i)';

    % --- 1. Update the robot pose (PreservePlot: false) ---
    show(robot, currentQ, ...
        'Visuals','on', ...
        'Collisions','off', ...
        'Frames','off', ...      
        'PreservePlot', false, ... 
        'Parent', ax);

    % --- 2. CRITICAL: Restore Axis Limits and View ---
    xlim([-1 1]); 
    ylim([-1 1]); 
    zlim([0 1]);
    % view(ax, 135, 25); 
    % axis equal;


    current_q_joint = currentQ(2); % Get the angle of the second joint


    angular_change = abs(current_q_joint - q_joint_sampled_last);


    if angular_change >= ANGLE_THRESHOLD_RAD

        T = getTransform(robot, currentQ, endEffectorName);
        currentPos = T(1:3, 4);


        hPoint = scatter3(ax, ...
            currentPos(1), currentPos(2), currentPos(3), ...
            'filled', 'MarkerFaceColor', 'b', 'SizeData', 75, ...
            'DisplayName', ['Point @ q2=', num2str(rad2deg(current_q_joint), '%.1f'), 'deg']); 


        q_joint_sampled_last = current_q_joint;


        pathHandles{pointIndex} = hPoint;
        pointIndex = pointIndex + 1;
    end

    for j = 1:length(pathHandles)
        % Note: The scatter3 call in Step 3 already places the newest point.
        % This loop ensures that the older points are also redrawn after 
        % the main 'show(robot, ...)' call clears them.

    end

    drawnow
end


q = [0.65 -pi/2 0 0]; 
qStart = q;
qGoal  = q; 

qGoal(1) = 0.1;     
qGoal(2) = -pi/2;
qGoal(3) = 0;
qGoal(4) = 0;

% Animation of the desired trajectory 
nSteps = 100;
[qMatrix,~,~] = trapveltraj([qStart; qGoal]', nSteps);



q_pos_sampled_last = qMatrix(1, 1); 

pathHandles = {}; 
pointIndex = 1;
for i = 1:nSteps
    
    currentQ = qMatrix(:,i)';
    
    % --- 1. Update the robot pose (PreservePlot: false) ---
    show(robot, currentQ, ...
        'Visuals','on', ...
        'Collisions','off', ...
        'Frames','off', ...      
        'PreservePlot', false, ... 
        'Parent', ax);
    

    xlim([-1 1]); 
    ylim([-1 1]); 
    zlim([0 1]);
    % view(ax, 135, 25); 
    % axis equal;
    
    current_q_pos= currentQ(1); % Get the angle of the second joint
    

    pos_change = abs(current_q_pos - q_pos_sampled_last);
    
    
    if pos_change >= 0.05
        

        T = getTransform(robot, currentQ, endEffectorName);
        currentPos = T(1:3, 4);

        hPoint = scatter3(ax, ...
            currentPos(1), currentPos(2), currentPos(3), ...
            'filled', 'MarkerFaceColor', 'b', 'SizeData', 75, ...
            'DisplayName', ['Point @ q2=', num2str(rad2deg(current_q_pos), '%.1f'), 'deg']); 

        q_pos_sampled_last = current_q_pos;
        

        pathHandles{pointIndex} = hPoint;
        pointIndex = pointIndex + 1;
    end


    drawnow
end






q = [0.1 -pi/2 0 0]; 
qStart = q;
qGoal  = q; 

qGoal(1) = 0.1;     
qGoal(2) = pi/2;
qGoal(3) = 0;
qGoal(4) = 0;

% Animation of the desired trajectory 
nSteps = 100;
[qMatrix,~,~] = trapveltraj([qStart; qGoal]', nSteps);



for i = 1:nSteps
    
    currentQ = qMatrix(:,i)';

    show(robot, currentQ, ...
        'Visuals','on', ...
        'Collisions','off', ...
        'Frames','off', ...      
        'PreservePlot', false, ... 
        'Parent', ax);
    

    xlim([-1 1]); 
    ylim([-1 1]); 
    zlim([0 1]);
    % view(ax, 135, 25); 
    % axis equal;
    
    current_q_joint = currentQ(2); % Get the angle of the second joint
    
    % Calculate the absolute angular change since the last plotted point
    angular_change = abs(current_q_joint - q_joint_sampled_last);
    
    % Check if the change exceeds the threshold (10 degrees)
    if angular_change >= ANGLE_THRESHOLD_RAD
        

        T = getTransform(robot, currentQ, endEffectorName);
        currentPos = T(1:3, 4);
        

        hPoint = scatter3(ax, ...
            currentPos(1), currentPos(2), currentPos(3), ...
            'filled', 'MarkerFaceColor', 'b', 'SizeData', 75, ...
            'DisplayName', ['Point @ q2=', num2str(rad2deg(current_q_joint), '%.1f'), 'deg']); 
        

        q_joint_sampled_last = current_q_joint;
        

        pathHandles{pointIndex} = hPoint;
        pointIndex = pointIndex + 1;
    end


    drawnow
end




q = [0.1 pi/2 0 0]; 
qStart = q;
qGoal  = q; 

qGoal(1) = 0.65;     
qGoal(2) = pi/2;
qGoal(3) = 0;
qGoal(4) = 0;

% Animation of the desired trajectory 
nSteps = 100;
[qMatrix,~,~] = trapveltraj([qStart; qGoal]', nSteps);


q_pos_sampled_last = qMatrix(1, 1); 


pathHandles = {}; 
pointIndex = 1;
for i = 1:nSteps
    
    currentQ = qMatrix(:,i)';
    

    show(robot, currentQ, ...
        'Visuals','on', ...
        'Collisions','off', ...
        'Frames','off', ...      
        'PreservePlot', false, ... 
        'Parent', ax);

    xlim([-1 1]); 
    ylim([-1 1]); 
    zlim([0 1]);
    % view(ax, 135, 25); 
    % axis equal;
    

    current_q_pos= currentQ(1); % Get the angle of the second joint
    
    % Calculate the absolute angular change since the last plotted point
    pos_change = abs(current_q_pos - q_pos_sampled_last);
    
    
    if pos_change >= 0.05

        T = getTransform(robot, currentQ, endEffectorName);
        currentPos = T(1:3, 4);
        

        hPoint = scatter3(ax, ...
            currentPos(1), currentPos(2), currentPos(3), ...
            'filled', 'MarkerFaceColor', 'b', 'SizeData', 75, ...
            'DisplayName', ['Point @ q2=', num2str(rad2deg(current_q_pos), '%.1f'), 'deg']); 
        
        % C. Update the last sampled joint angle to the current angle
        q_pos_sampled_last = current_q_pos;
        
        % D. Store the handle (for completeness, though it's replotted below)
        pathHandles{pointIndex} = hPoint;
        pointIndex = pointIndex + 1;
    end

    for j = 1:length(pathHandles)
        % Note: The scatter3 call in Step 3 already places the newest point.
        % This loop ensures that the older points are also redrawn after 
        % the main 'show(robot, ...)' call clears them.

    end

    drawnow
end
