%% Clean start
clear; clc; close all;

%% --------- USER SETTINGS ----------
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

% Manually set fixed limits
xlim([-1 1]); 
ylim([-1 1]); 
zlim([0 1]);

q = [0.65 0 0 0]; 
qStart = q;
qGoal  = q;

 
qGoal(2) = -pi/2;
qGoal(3) = -3*pi/4;
qGoal(4) = -pi/4;

qMiddle = qGoal; 

qMiddle(1) = 0.2; 

qFinish = q; 
qFinish(1) =0.2; 




% Animation of the desired trajectory 
nSteps = 150;
[qMatrix,~,~] = trapveltraj([qStart; qGoal; qMiddle; qFinish]', nSteps);


for i = 1:nSteps
    show(robot, qMatrix(:,i)', ...
        'Visuals','on', ...
        'Collisions','off', ...
        'Frames','off', ...
        'PreservePlot', false, ...
        'Parent', ax);

    drawnow
end