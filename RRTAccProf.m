%--------------------------------------------------------------------------
%            Parthasarathy Nadarajan
%            CARISSMA & Technische Hochschule Ingolstadt (THI)
%            Path Planning using RRT and Predicted Occupancy Grids
%            Using three different acceleration profiles
%--------------------------------------------------------------------------
clc; clearvars; close all;

%% Define the configuration space
load('POG_scenario');

% Rotate the occupancy grid
gridConverted = rot90(predictedOccupancyGrid, 3);

% Plot the occupied cells
[a, b] = find(gridConverted(:,:,1) >0.01);
figure(1);
axis([1 80 1 80])
s1 = scatter(a, b, 'g', 's', 'filled'); hold on
figure(2);
axis([1 40 1 40])
s2 = scatter(a*0.5, b*0.5, 'g', 's', 'filled'); hold on

%% EGO vehicle data
EGO.xCG = 30; % m
EGO.yCG = 20; % m
EGO.vel = 10; % m/s
EGO.psi = 180; % degrees
EGO.ax = 0; % m/s^2
EGO.ay = 0; % m/s^2
EGO.lf = 2; % m
EGO.lr = 2; % m
EGO.w = 2; % m
EGO.steerMax = 3; % degrees

%% Parameters for RRT algorithm
param.RRTSampleTime = 0.1; % new node sampling interval
param.simulationSampleTime = 0.02; % simulation step
param.maxIter = 2000;
param.minThresh = 0.1; % minimum probability of occupancy
param.stopThresh = 0.5; % distance around the goal to stop simulation
param.plotGraph = false;
saveImage = false;

%% Define the start and goal point
% Prediction time
tPred = 2; % seconds

% Start point in terms of grid
p_start = [round(EGO.xCG/0.5), round(EGO.yCG/0.5)];
% Start point in terms of position
p_pos_start(1) = EGO.xCG;
p_pos_start(2) = EGO.yCG;

% EGO position after 2 seconds will be the goal position (without any change in the state)
distMoved = EGO.vel*tPred + 0.5*EGO.ax*tPred^2;
tx_pos = distMoved*cosd(EGO.psi);
ty_pos = distMoved*sind(EGO.psi);
p_pos_goal(1) = EGO.xCG+tx_pos;
p_pos_goal(2) = EGO.yCG+ty_pos;
% Goal point in terms of grid
p_goal = [round(p_pos_goal(1)/0.5), round(p_pos_goal(2)/0.5)];

% Plot the start and the goal point
figure(1)
scatter(p_start(1), p_start(2), 'r', 'filled', 'd');
scatter(p_goal(1), p_goal(2), 'b', 'filled', 'd');
legend('Occupied', 'Start', 'Goal');
legend('AutoUpdate', 'off')
figure(2);
scatter(p_pos_start(1), p_pos_start(2), 'r', 'filled', 'd');
scatter(p_pos_goal(1), p_pos_goal(2), 'b', 'filled', 'd');
legend('Occupied', 'Start', 'Goal');
legend('AutoUpdate', 'off')

%% Initialise the tree
% RRT with constant velocity profile
rrtConstantVel = [];
rrtConstantVel = AddNode(rrtConstantVel, p_start, p_pos_start, 0, EGO.psi, EGO.vel, 1, 1); %0 - root node; 1 - simulationStep; 1 - terminal node;

% RRT with braking acceleration profile
rrtBraking = [];
rrtBraking = AddNode(rrtBraking, p_start, p_pos_start, 0, EGO.psi, EGO.vel, 1, 1); %0 - root node; 1 - simulationStep; 1 - terminal node;

% RRT with current state acceleration profile
rrtCurrentState = [];
rrtCurrentState = AddNode(rrtCurrentState, p_start, p_pos_start, 0, EGO.psi, EGO.vel, 1, 1); %0 - root node; 1 - simulationStep; 1 - terminal node;

%% Path planning
% Path planning with the current state
tic
[rrtCurrentState, iterCurrentState] = pathPlanning(p_goal, p_pos_goal, gridConverted, rrtCurrentState, EGO, param);
timeTakenCS = toc;

% Path planning with constant velocity profile
tic
EGO.ax = 0;
[rrtConstantVel, iterConstantVel] = pathPlanning(p_goal, p_pos_goal, gridConverted, rrtConstantVel, EGO, param);
timeTakenCV = toc;

% Path planning with braking profile
tic
EGO.ax = -5;
[rrtBraking, iterBraking] = pathPlanning(p_goal, p_pos_goal, gridConverted, rrtBraking, EGO, param);
timeTakenBr = toc;

%% Extracting the path from tree
% Constant velocity profile------------------------------------------------
i = length(rrtConstantVel);
% Select the node that is closest to the goal as well as a terminal node
for j = 1:i
    dist = sqrt((rrtConstantVel(j).node.p(1) -p_goal(1))^2 + (rrtConstantVel(j).node.p(2)-p_goal(2))^2);
    if (j == 1) || (dist < mindist) && (rrtConstantVel(j).node.terminalNode == 1) && (rrtConstantVel(j).node.sampleTime == 21)
        mindist = dist;
        imin = j;
    end
end

% Plotting the final position
figure(1);
scatter(rrtConstantVel(imin).node.p(1), rrtConstantVel(imin).node.p(2), 'm', 'filled', 'd', 'LineWidth', 2);
legend('Occupied', 'Start', 'Goal', 'Final point with CV')
figure(2);
scatter(rrtConstantVel(imin).node.p_pos(1), rrtConstantVel(imin).node.p_pos(2), 'm', 'filled', 'd', 'LineWidth', 2);
legend('Occupied', 'Start', 'Goal', 'Final point with CV')
% Extracting the required information from the RRT
P_Const_Vel = rrtConstantVel(imin).node.p';
P_Const_Vel_pos = rrtConstantVel(imin).node.p_pos';
P_Const_Vel_angle = rrtConstantVel(imin).node.angPrev;
P_Const_Vel_sampleTime = rrtConstantVel(imin).node.sampleTime;
P_Const_Vel_vel = rrtConstantVel(imin).node.vel;
j = imin;

while 1
    j = rrtConstantVel(j).node.iPrev;
    if j == 0
        break
    end
    P_Const_Vel = [rrtConstantVel(j).node.p' P_Const_Vel];
    P_Const_Vel_pos = [rrtConstantVel(j).node.p_pos' P_Const_Vel_pos];
    P_Const_Vel_angle = [rrtConstantVel(j).node.angPrev P_Const_Vel_angle];
    P_Const_Vel_sampleTime = [rrtConstantVel(j).node.sampleTime P_Const_Vel_sampleTime];
    P_Const_Vel_vel = [rrtConstantVel(imin).node.vel P_Const_Vel_vel];
end


% Braking profile----------------------------------------------------------
i = length(rrtBraking);

% Select the node that is closest to the goal as well as a terminal node
for j = 1:i
    dist = sqrt((rrtBraking(j).node.p(1) -p_goal(1))^2 + (rrtBraking(j).node.p(2)-p_goal(2))^2);
    if (j == 1) || (dist < mindist) && (rrtBraking(j).node.terminalNode == 1)
        mindist = dist;
        imin = j;
    end
end

% Plotting the final position
figure(1);
scatter(rrtBraking(imin).node.p(1), rrtBraking(imin).node.p(2), 'c', 'filled', 'd', 'LineWidth', 2);
legend('Occupied', 'Start', 'Goal', 'Final point with CV', 'Final point with Brake');
figure(2);
scatter(rrtBraking(imin).node.p_pos(1), rrtBraking(imin).node.p_pos(2), 'c', 'filled', 'd', 'LineWidth', 2);
legend('Occupied', 'Start', 'Goal', 'Final point with CV', 'Final point with Brake');

% Extracting the required information from the RRT
P_Braking = rrtBraking(imin).node.p';
P_Braking_pos = rrtBraking(imin).node.p_pos';
P_Braking_angle = rrtBraking(imin).node.angPrev;
P_Braking_sampleTime = rrtBraking(imin).node.sampleTime;
P_Braking_vel = rrtBraking(imin).node.vel;
j = imin;

while 1
    j = rrtBraking(j).node.iPrev;
    if j == 0
        break
    end
    P_Braking = [rrtBraking(j).node.p' P_Braking];
    P_Braking_pos = [rrtBraking(j).node.p_pos' P_Braking_pos];
    P_Braking_angle = [rrtBraking(j).node.angPrev P_Braking_angle];
    P_Braking_sampleTime = [rrtBraking(j).node.sampleTime P_Braking_sampleTime];
    P_Braking_vel = [rrtBraking(j).node.vel P_Braking_vel];
end

%% Plot the unsmoothed path

% Constant Velocity profile------------------------------------------------
if length(P_Const_Vel) < 3
    disp('Could not find valid path with constant velocity profile');
else
    fprintf('Time taken for the RRT algorithm to find a path using constant velocity profile is %d \n', timeTakenCV);
    fprintf('The number of iterations taken is %d \n', iterConstantVel-1);
    saveDir = 'ConstantVelocity';
    plotGraphWithOcc(P_Const_Vel, P_Const_Vel_pos, P_Const_Vel_angle, P_Const_Vel_sampleTime, gridConverted, EGO, saveImage, saveDir)
    plotRRTNode(rrtConstantVel, 'm')
end

% Braking profile----------------------------------------------------------
if length(P_Braking) < 3
    disp('Could not find valid path with braking profile');
else
    fprintf('Time taken for the RRT algorithm to find a path using braking profile is %d \n', timeTakenBr);
    fprintf('The number of iterations taken is %d \n', iterBraking-1);
    saveDir = 'BrakeProfile';
    plotGraphWithOcc(P_Braking, P_Braking_pos, P_Braking_angle, P_Braking_sampleTime, gridConverted, EGO, saveImage, saveDir)
    plotRRTNode(rrtBraking, 'c')
end

%% Using the controller to follow the unsmoothed path


%% Additional functions
% Function - Add node to the tree------------------------------------------
function rrt = AddNode(rrt, p, p_pos, iPrev, p_angle, p_vel, sampleTime, termNode)
node.p = p;
node.p_pos = p_pos;
node.iPrev = iPrev;
node.angPrev = p_angle;
node.vel = p_vel;
node.sampleTime = sampleTime;
node.terminalNode = termNode;
if length(rrt) >= 1
    rrt(iPrev).node.terminalNode = 0;
end
rrt(end+1).node = node;
end

% Function - Path planning-------------------------------------------------
function [rrt, iter] = pathPlanning(p_goal, p_pos_goal, gridConverted, rrt, EGO, param)
iter = 1;
sampleTime = 1;

while iter <= param.maxIter
    
    % Choose a random point and sometime biased based on the goal
    if rem(iter, 50) == 0
        p_rand(1) = p_goal(1);
        p_rand(2) = p_goal(2);
    else
        p_rand(1) = randperm(80, 1);
        p_rand(2) = randperm(80, 1);
    end
    
    % Skip to next iteration if the vertex is not valid based on occupancy
    probOccupancy = gridConverted(p_rand(1), p_rand(2), sampleTime);
    if probOccupancy > param.minThresh
        iter = iter+1;
        continue
    end
    
    % Conversion from grid to position
    p_pos_rand(2) = p_rand(2)*0.5;
    p_pos_rand(1) = p_rand(1)*0.5;
    
    % Plot the selected random node
    if param.plotGraph
        [a, b] = find(gridConverted(:,:,sampleTime) >0.01);
        figure(1)
        plotObject(1) = scatter(p_rand(1), p_rand(2), 'ro');
        plotObject(2) = scatter(a, b, 'g', 's', 'filled');
        figure(2)
        plotObject(3) = scatter(p_pos_rand(1), p_pos_rand(2), 'ro');
        plotObject(4) = scatter(a*0.5, b*0.5, 'g', 's', 'filled');
    end
    
    
    % Select the nearest vertex to the chosen random vertex
    for i = 1:length(rrt)
        
        % Heuristic 1 - Calculate the distance between the vertices
        dist =sqrt((rrt(i).node.p(1) -p_rand(1))^2 + (rrt(i).node.p(2)-p_rand(2))^2);
        
        % Heuristic 2 - Angle deviation required
        p_angle = atan2d(p_pos_rand(2)-rrt(i).node.p(2)*0.5, p_pos_rand(1)-rrt(i).node.p(1)*0.5)+360*(p_pos_rand(2)-rrt(i).node.p(2)*0.5<0);
        l_angle = rrt(i).node.angPrev;
        
        if (p_angle >=0 && p_angle <=90 && l_angle >= 270 && l_angle <=360)
            angle_diff = p_angle + (360-l_angle);
        elseif (l_angle >=0 && l_angle <=90 && p_angle >= 270 && p_angle <=360)
            angle_diff = l_angle + (360-p_angle);
        else
            angle_diff = l_angle-p_angle;
        end
        
        % Cost
        cost = dist+abs(angle_diff)/360;
        if (i == 1) || (cost < mincost)
            mincost = cost;
            imin = i;
            l = rrt(imin).node.p;
            l_pos = rrt(imin).node.p_pos;
            min_l_angle = rrt(imin).node.angPrev;
            min_p_angle = p_angle;
            min_angle_diff = angle_diff;
            min_p_vel = rrt(imin).node.vel;
            sampleTime = rrt(imin).node.sampleTime;
        end
    end
    
    % Convert grid to position
    p_pos(2) = l_pos(2);
    p_pos(1) = l_pos(1);
    
    % Plot the selected nearest vertex
    if param.plotGraph
        figure(1)
        plotObject(5) = scatter(l(1), l(2), 'b*');
        figure(2)
        plotObject(6) = scatter(p_pos(1), p_pos(2), 'b*');
    end
    
    % Find a path from the selected nearest vertex to the chosen random vertex
    [col, p_step, p_pos_step, p_angle, p_vel] = InCollision_Edge(p_pos, p_pos_rand, min_l_angle, min_p_angle, min_p_vel, min_angle_diff, gridConverted, EGO, param, sampleTime);
    
    % Skip to next iteration if it is not valid edge
    if col == 1
        iter = iter+1;
        if param.plotGraph
            delete(plotObject);
        end
        continue
    end
    
    % Plot the selected nearest node
    if param.plotGraph
        figure(2);
        plotObject(7) = scatter(p_step(1), p_step(2), 'k', 'filled');
        figure(3);
        plotObject(8) = scatter(p_pos_step(1), p_pos_step(2), 'k', 'filled');
    end
    
    % Increment the sample time by 1 when a valid vertex is found
    sampleTime = sampleTime+1;
    
    % Add the valid vertex to the tree
    rrt = AddNode(rrt, p_step, p_pos_step, imin, p_angle, p_vel, sampleTime, 1);
    
    % Calculating the distance to the goal
    dist = norm(p_pos_step-p_pos_goal);
    
    % Considering the distance to the goal
    if (dist <= param.stopThresh) || (iter >= param.maxIter) || (sampleTime == 21) || (p_vel == 0)
        break
    end
    iter = iter+1;
    if param.plotGraph
        delete(plotObject)
    end
end
end

% Function - Plotting the RRT Nodes----------------------------------------
function plotRRTNode(rrt, color)
for i = 1:length(rrt)
    figure(1);
    scatter(rrt(i).node.p(1), rrt(i).node.p(2), color, 'filled');
    figure(2);
    scatter(rrt(i).node.p(1)*0.5, rrt(i).node.p(2)*0.5, color, 'filled');
end
end

% Function - Plotting the unsmoothed path----------------------------------
function plotGraphWithOcc(P, P_pos, P_angle, P_sampleTime, gridConverted, EGO, saveImage, saveDir)
for i = 2:length(P)
    figure(1);
    plot([P(1,i);P(1,i-1)],[P(2,i);P(2,i-1)],'b','LineWidth',1);
    
    if saveImage
    figure(2);
    % Plot of the POG for the corresponding time instance
    [a, b] = find(gridConverted(:,:,P_sampleTime(i)) >0.0);
    for j = 1:length(a)
        s2 = scatter(a(j)*0.5, b(j)*0.5, 10, gridConverted(a(j), b(j), P_sampleTime(i)), 'filled'); hold on
    end
    % Plot of the selected point
    scatter(P_pos(1,i),P_pos(2,i),'ko', 'filled');
    end
    figure(2);
    plot([P_pos(1,i);P_pos(1,i-1)],[P_pos(2,i);P_pos(2,i-1)],'b','LineWidth',1);
    
    % Plotting the vehicle
    if saveImage
    ex=cosd(P_angle(i));
    ey=sind(P_angle(i));
    exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
    eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
    P_plot(1)=P(1,i)*0.5+ex*(EGO.lf)+EGO.w/2*exOrtho;
    P_plot(2)=P(2,i)*0.5+ey*(EGO.lf)+EGO.w/2*eyOrtho;
    P_plot(3)=P(1,i)*0.5+ex*(EGO.lf)-EGO.w/2*exOrtho;
    P_plot(4)=P(2,i)*0.5+ey*(EGO.lf)-EGO.w/2*eyOrtho;
    P_plot(5)=P(1,i)*0.5-ex*(EGO.lr)+EGO.w/2*exOrtho;
    P_plot(6)=P(2,i)*0.5-ey*(EGO.lr)+EGO.w/2*eyOrtho;
    P_plot(7)=P(1,i)*0.5-ex*(EGO.lr)-EGO.w/2*exOrtho;
    P_plot(8)=P(2,i)*0.5-ey*(EGO.lr)-EGO.w/2*eyOrtho;
    p(1) = plot([P_plot(1) P_plot(3)], [P_plot(2) P_plot(4)], 'Color', 'b', 'LineWidth', 2);
    p(2) = plot([P_plot(1) P_plot(5)], [P_plot(2) P_plot(6)], 'Color', 'b');
    p(3) = plot([P_plot(3) P_plot(7)], [P_plot(4) P_plot(8)], 'Color', 'b');
    p(4) = plot([P_plot(7) P_plot(5)], [P_plot(8) P_plot(6)], 'Color', 'b');
    mkdir(saveDir)
    saveName = [num2str(i) '.png'];
    saveas(gcf, saveName);
    delete(p);
    delete(s2);
    close(figure(2))
    end
end
end
