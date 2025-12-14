<h2 align="left"> Final Project</h2>
<h1 align="center"> P3R Book Sorting Robot</h1>
<h2 align="center">Team A-1</h2>
<h2 align="left">Project details:</h2>
<details>
<summary> About the Robot

</summary>

</details>

<details>
<summary> Simulations

</summary>

https://github.com/user-attachments/assets/466b8830-91f7-4014-bd22-bce9394509c4


%%          BASIC SIMULATION 

clear 

clc



L1 = 0; 

L2 = 5; 

L3 = 5; 

L4 = 4; 



% theta d a alpha

DH = [0 0 0 0; 0 0 L1 0 ; 0 0 L2 0 ; 0 0 L3 0 ; 0 0 L4 0]; 





L(1) = Link('prismatic', 'theta', DH(1,1), 'a', DH(1,3), 'alpha', DH(1,4), 'modified');

L(2) = Link('revolute', 'd', DH(2,2), 'a', DH(2,3), 'alpha', DH(2,4), 'modified');

L(3) = Link('revolute', 'd', DH(3,2), 'a', DH(3,3), 'alpha', DH(3,4), 'modified');

L(4) = Link('revolute', 'd', DH(4,2), 'a', DH(4,3), 'alpha', DH(4,4), 'modified');

L(5) = Link('revolute', 'd', DH(5,2), 'a', DH(5,3), 'alpha', DH(5,4), 'modified');



bobot = SerialLink(L, 'name', 'bobot'); 





qi = [6 0 pi/6 0 0]; 

qd = [9 2*pi -pi/6 0 0]; 

q0 = [9 0 pi/6 0 0]; 



Ti = fkine(bobot, qi);

Td = fkine(bobot, qd);



% bobot.plot(qi, 'workspace', [-20, 20, -20, 20, -5, 20])



% Number of steps in animation

steps = 500;



% Generate trajectory from qi to qd

q_traj = jtraj(qi, q0, steps);

q_traj2 = jtraj(q0, qd, steps);



% Open a figure

figure;







% bobot.plot(qi, 'workspace', [-20 20 -20 20 -5 20])



% Animate



bobot.plot(q_traj, 'workspace', [-20 20 -20 20 -10 20], 'delay', 0.00000001);   % <-- makes animation as fast as possible

bobot.plot(q_traj2, 'workspace', [-20 20 -20 20 -10 20], 'delay', 0.00000001);

https://github.com/user-attachments/assets/0b08ce72-e509-4a36-8629-4c3a73730fc9

%% Clean start
clear; clc; close all;

%% --------- USER SETTINGS ----------
urdfFile = "URDF_Second.URDF";   
% ----------------------------------
robotFolder = fileparts(which(urdfFile));
addpath(genpath(robotFolder));
setenv('URDF_Second.URDF', robotFolder);

robot = importrobot(urdfFile);

%% Recommended settings
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


% for i = 1:nSteps
%     show(robot, qMatrix(:,i)', ...
%         'Visuals','on', ...
%         'Collisions','off', ...
%         'Frames','off', ...
%         'PreservePlot', false, ...
%         'Parent', ax);
% 
%     drawnow
% end



% --- Configuration variables ---
endEffectorName = 'link4'; % Assuming Link4 is the end-effector
nSteps = 100;
% Define the angular threshold in radians (10 degrees)
ANGLE_THRESHOLD_RAD = deg2rad(5); 

% Initialize a variable to track the joint angle at the last plotted point
% We use the initial angle of the sampling joint (Joint 2 in this case).
q_joint_sampled_last = qMatrix(1, 2); 

% Initialize a cell array to store the plot handles for the path trace
pathHandles = {}; 
pointIndex = 1;



%% ----------------- UPDATED ANIMATION LOOP (10-Degree Sampling) -----------------
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

    % --- 3. Sampling Logic (Plot point every 10 degrees of Joint 2 movement) ---
    current_q_joint = currentQ(2); % Get the angle of the second joint

    % Calculate the absolute angular change since the last plotted point
    angular_change = abs(current_q_joint - q_joint_sampled_last);

    % Check if the change exceeds the threshold (10 degrees)
    if angular_change >= ANGLE_THRESHOLD_RAD

        % A. Calculate End-Effector Position
        T = getTransform(robot, currentQ, endEffectorName);
        currentPos = T(1:3, 4);

        % B. Plot a new point (scatter3)
        hPoint = scatter3(ax, ...
            currentPos(1), currentPos(2), currentPos(3), ...
            'filled', 'MarkerFaceColor', 'b', 'SizeData', 75, ...
            'DisplayName', ['Point @ q2=', num2str(rad2deg(current_q_joint), '%.1f'), 'deg']); 

        % C. Update the last sampled joint angle to the current angle
        q_joint_sampled_last = current_q_joint;

        % D. Store the handle (for completeness, though it's replotted below)
        pathHandles{pointIndex} = hPoint;
        pointIndex = pointIndex + 1;
    end

    % --- 4. Re-plot all previously stored points ---
    % Since 'show(..., PreservePlot, false)' clears the axis, we must 
    % redraw all the previously placed blue points in every step.
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




% Initialize a variable to track the joint angle at the last plotted point
% We use the initial angle of the sampling joint (Joint 2 in this case).
q_pos_sampled_last = qMatrix(1, 1); 

% Initialize a cell array to store the plot handles for the path trace
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
    
    % --- 3. Sampling Logic (Plot point every 10 degrees of Joint 2 movement) ---
    current_q_pos= currentQ(1); % Get the angle of the second joint
    
    % Calculate the absolute angular change since the last plotted point
    pos_change = abs(current_q_pos - q_pos_sampled_last);
    
    
    if pos_change >= 0.05
        
        % A. Calculate End-Effector Position
        T = getTransform(robot, currentQ, endEffectorName);
        currentPos = T(1:3, 4);
        
        % B. Plot a new point (scatter3)
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
    
    % --- 4. Re-plot all previously stored points ---
    % Since 'show(..., PreservePlot, false)' clears the axis, we must 
    % redraw all the previously placed blue points in every step.
    for j = 1:length(pathHandles)
        % Note: The scatter3 call in Step 3 already places the newest point.
        % This loop ensures that the older points are also redrawn after 
        % the main 'show(robot, ...)' call clears them.

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
    
   % --- 3. Sampling Logic (Plot point every 10 degrees of Joint 2 movement) ---
    current_q_joint = currentQ(2); % Get the angle of the second joint
    
  % Calculate the absolute angular change since the last plotted point
    angular_change = abs(current_q_joint - q_joint_sampled_last);
    
   % Check if the change exceeds the threshold (10 degrees)
    if angular_change >= ANGLE_THRESHOLD_RAD
        
  % A. Calculate End-Effector Position
        T = getTransform(robot, currentQ, endEffectorName);
        currentPos = T(1:3, 4);
        
   % B. Plot a new point (scatter3)
        hPoint = scatter3(ax, ...
            currentPos(1), currentPos(2), currentPos(3), ...
            'filled', 'MarkerFaceColor', 'b', 'SizeData', 75, ...
            'DisplayName', ['Point @ q2=', num2str(rad2deg(current_q_joint), '%.1f'), 'deg']); 
        
   % C. Update the last sampled joint angle to the current angle
        q_joint_sampled_last = current_q_joint;
        
   % D. Store the handle (for completeness, though it's replotted below)
        pathHandles{pointIndex} = hPoint;
        pointIndex = pointIndex + 1;
    end
    
   % --- 4. Re-plot all previously stored points ---
    % Since 'show(..., PreservePlot, false)' clears the axis, we must 
    % redraw all the previously placed blue points in every step.
    for j = 1:length(pathHandles)
        % Note: The scatter3 call in Step 3 already places the newest point.
        % This loop ensures that the older points are also redrawn after 
        % the main 'show(robot, ...)' call clears them.

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




% Initialize a variable to track the joint angle at the last plotted point
% We use the initial angle of the sampling joint (Joint 2 in this case).
q_pos_sampled_last = qMatrix(1, 1); 

% Initialize a cell array to store the plot handles for the path trace
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
    
  % --- 3. Sampling Logic (Plot point every 10 degrees of Joint 2 movement) ---
    current_q_pos= currentQ(1); % Get the angle of the second joint
    
  % Calculate the absolute angular change since the last plotted point
    pos_change = abs(current_q_pos - q_pos_sampled_last);
    
    
  if pos_change >= 0.05
        
  % A. Calculate End-Effector Position
        T = getTransform(robot, currentQ, endEffectorName);
        currentPos = T(1:3, 4);
        
   % B. Plot a new point (scatter3)
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
    
  % --- 4. Re-plot all previously stored points ---
    % Since 'show(..., PreservePlot, false)' clears the axis, we must 
    % redraw all the previously placed blue points in every step.
    for j = 1:length(pathHandles)
        % Note: The scatter3 call in Step 3 already places the newest point.
        % This loop ensures that the older points are also redrawn after 
        % the main 'show(robot, ...)' call clears them.

   end

  drawnow
end
</details>

<details>
<summary> Mathematical Models
</summary>
  
#### DH-Parameters
  
#### Jacobian
</details>

</details>


<details>
<summary> CAD Models

</summary>

</details>

<details>
<summary>Team Members

</summary>

🤖 Federico Alonsexo

👩‍🔧 Giovana Bogado

👨‍🏭 Andres Fernandez

🛠️ Facundo Mendoza

</details>


