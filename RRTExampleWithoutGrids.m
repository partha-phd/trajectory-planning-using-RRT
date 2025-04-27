%--------------------------------------------------------------------------
%            Parthasarathy Nadarajan
%            CARISSMA & Technische Hochschule Ingolstadt (THI)
%            Path Planning using RRT 
%--------------------------------------------------------------------------
clc; clearvars; close all;

%% Define the configuration space
load('POG_scenario');
% Rotate the occupancy grid
gridConverted = rot90(predictedOccupancyGrid, 3);

%% EGO vehicle data
EGO.vel = 5; % m/s
EGO.psi = 0; % degrees
EGO.ax = 0; % m/s^2
EGO.ay = 0; % m/s^2
EGO.xCG = 10; % m
EGO.yCG = 20; % m
EGO.lf = 2; % m
EGO.lr = 2; % m
EGO.w = 2; % m

%% Define the start and goal point
% Prediction time for the path planning algorithm
predictionTime = 4; % s

p_start = [EGO.xCG/0.5, EGO.yCG/0.5]; % grid position

% Start point in terms of position
p_pos_start(1) = EGO.xCG;
p_pos_start(2) = EGO.yCG;

% EGO position after 2 seconds will be the goal position
distMoved = EGO.vel*predictionTime; 
tx_pos = distMoved*cosd(EGO.psi-10);
ty_pos = distMoved*sind(EGO.psi-10);
p_pos_goal(1) = EGO.xCG+tx_pos;
p_pos_goal(2) = EGO.yCG+ty_pos;

p_goal = [round(p_pos_goal(1)/0.5), round(p_pos_goal(2)/0.5)]; % grid position

% Plot the start and the goal point
figure(2)
scatter(p_start(1), p_start(2), 'r', 'filled', 'd'); hold on
scatter(p_goal(1), p_goal(2), 'b', 'filled', 'd');
axis([1 80 1 80]);
title('Grid position');
legend('Start', 'Goal');
legend('AutoUpdate', 'off')

figure(3);
scatter(p_pos_start(1), p_pos_start(2), 'r', 'filled', 'd'); hold on
scatter(p_pos_goal(1), p_pos_goal(2), 'b', 'filled', 'd');
axis([1 40 1 40]);
title('Position in X and Y');
legend('Start', 'Goal');
legend('AutoUpdate', 'off')

%% Parameters for RRT algorithm
param.maxIter = 2000;
param.minThresh = 1.0; % minimum probability of occupancy
RRTSampleTime = 0.1;
simulationStepTime = 0.02;
param.minRes = EGO.vel*simulationStepTime; % steps in between RRT
param.minDist = EGO.vel*RRTSampleTime; % distance constrained based on RRT sample time
param.stopThresh = 0.5; % distance around the goal to stop simulation
param.plotGraph = true;
param.minAngle = 40; % constraints on the steering

%% Initialise the tree
rrt = {};
rrt = AddNode(rrt, p_start, p_pos_start, 0, EGO.psi, 1, 1); %0 - root node, %1 - simulationTime is 1 % 1 - terminal node

%% Path planning
tic
[rrt, iter] = pathPlanning(p_goal, p_pos_goal, rrt, EGO, gridConverted, param);
timeTaken = toc;

%% Extracting the path from tree
i = length(rrt);
% Select the node that is closest to the goal as well as a terminal node
for j = 1:i
    dist = sqrt((rrt{j}.p(1) -p_goal(1))^2 + (rrt{j}.p(2)-p_goal(2))^2);
    if (j == 1) || (dist < mindist) && (rrt{j}.terminalNode == 1)
        mindist = dist;
        imin = j;
    end
end

figure(2);
scatter(rrt{imin}.p(1), rrt{imin}.p(2), 'g', 'filled', 'd', 'LineWidth', 2);
P = rrt{imin}.p';
P_angle = rrt{imin}.angPrev;
j = imin;
while 1
    j = rrt{j}.iPrev;
    if j == 0
        break
    end
    P = [rrt{j}.p' P];
    P_pos = P*0.5;
    P_angle = [rrt{j}.angPrev P_angle];
end

%% Plot the unsmoothed path
if isempty(P)
    disp('Could not find valid path');
else
    fprintf('Time taken for the RRT algorithm to find a path is %d', timeTaken);
    fprintf('The number of iterations taken is %d', iter-1);
    
    for i = 2:length(P)
        figure(2);
        scatter(P(1,i),P(2,i),'ko', 'filled');
        plot([P(1,i);P(1,i-1)],[P(2,i);P(2,i-1)],'b','LineWidth',1);
        figure(3);
        scatter(P(1,i)*0.5,P(2,i)*0.5,'ko', 'filled');
        plot([P(1,i)*0.5;P(1,i-1)*0.5],[P(2,i)*0.5;P(2,i-1)*0.5],'b','LineWidth',1);
        % Plotting the vehicle
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
        plot([P_plot(1) P_plot(3)], [P_plot(2) P_plot(4)], 'Color', 'b', 'LineWidth', 2);
        plot([P_plot(1) P_plot(5)], [P_plot(2) P_plot(6)], 'Color', 'b');
        plot([P_plot(3) P_plot(7)], [P_plot(4) P_plot(8)], 'Color', 'b');
        plot([P_plot(7) P_plot(5)], [P_plot(8) P_plot(6)], 'Color', 'b');
    end
    for i = 1:length(rrt)
        if rrt{i}.terminalNode == 0
            figure(2);
            scatter(rrt{1,i}.p(1), rrt{1,i}.p(2), 'yo', 'filled');
            figure(3);
            scatter(rrt{1,i}.p(1)*0.5, rrt{1,i}.p(2)*0.5, 'yo', 'filled');
        else
            figure(2);
            scatter(rrt{1,i}.p(1), rrt{1,i}.p(2), 'mo', 'filled');
            figure(3);
            scatter(rrt{1,i}.p(1)*0.5, rrt{1,i}.p(2)*0.5, 'mo', 'filled');
        end
    end
end

%% Function - Path planning
function [rrt, iter] = pathPlanning(p_goal, p_pos_goal, rrt, EGO, gridConverted, param)
iter = 1;
sampleTime = 1;

while iter <= param.maxIter
    % Choose a random point and sometime biased based on the goal
    if rem(iter, 100) == 0
        p_rand(1) = p_goal(1);
        p_rand(2) = p_goal(2);
    else
        p_rand(1) = randperm(80, 1);
        p_rand(2) = randperm(80, 1);
    end
    
    % Skip to next iteration if the vertex is not valid based on occupancy
    probOccupancy = gridConverted(p_rand(1), p_rand(2), 1);
    if probOccupancy > param.minThresh
        iter = iter+1;
        continue
    end
    
    % Conversion from grid to position
    p_pos_rand(2) = p_rand(2)*0.5;
    p_pos_rand(1) = p_rand(1)*0.5;
    
    % Plot the selected random node
    if param.plotGraph
        figure(2)
        plotObject(1) = scatter(p_rand(1), p_rand(2), 'ro');
        figure(3)
        plotObject(3) = scatter(p_pos_rand(1), p_pos_rand(2), 'ro');
    end
    
    % Select the nearest vertex to the chosen random vertex
    for i = 1:length(rrt)
        
        % Heuristic 1 - Calculate the distance between the vertices
        dist =sqrt((rrt{i}.p(1) -p_rand(1))^2 + (rrt{i}.p(2)-p_rand(2))^2);
        
        % Heuristic 2 - Angle deviation required 
        p_angle = atan2d(p_pos_rand(2)-rrt{i}.p(2)*0.5, p_pos_rand(1)-rrt{i}.p(1)*0.5)+360*(p_pos_rand(2)-rrt{i}.p(2)*0.5<0);     
        l_angle = rrt{i}.angPrev; 
        
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
            l = rrt{i}.p;
            sampleTime = rrt{i}.sampleTime;
            min_angle_diff = angle_diff;
            min_p_angle = p_angle;
        end
    end
    
    if abs(min_angle_diff) > param.minAngle
        iter = iter+1;
        continue;
    end
    
    % Convert grid to position
    p_pos(2) = l(2)*0.5;
    p_pos(1) = l(1)*0.5;
    
    % Plot the selected nearest vertex
    if param.plotGraph
        figure(2)
        plotObject(5) = scatter(l(1), l(2), 'b*');
        figure(3)
        plotObject(6) = scatter(p_pos(1), p_pos(2), 'b*');
    end
    
    % Find a path from the selected nearest vertex to the chosen random vertex
    [col, p_step, p_pos_step] = InCollision_Edge(p_pos, min_p_angle, EGO, gridConverted, param);
    
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
    rrt = AddNode(rrt, p_step, p_pos_step, imin, min_p_angle, sampleTime, 1);
    
    % Calculating the distance to the goal
    dist = norm(p_pos_step-p_pos_goal);
    
    % Considering the distance to the goal
    if (dist <= param.stopThresh) || (iter >= param.maxIter)
        break
    end
    iter = iter+1;
    if param.plotGraph
        delete(plotObject)
    end
end
end

% Function - Add node to the tree------------------------------------------
function rrt = AddNode(rrt, p, p_pos, iPrev, p_angle, sampleTime, termNode)
node.p = p; % position in terms of grid
node.p_pos = p_pos; % position in X and Y
node.iPrev = iPrev; % parent node
node.angPrev = p_angle; % angle between the parent node and child node
node.sampleTime = sampleTime; % time instance at which the node was created
node.terminalNode = termNode; % 1 - terminal node; 0 - not terminal node
if length(rrt) >= 1
    rrt{1,iPrev}.terminalNode = 0;
end
rrt{end+1} = node;
end

% Function - Collision check along the edge--------------------------------
function [col, p_step, p_pos_step] = InCollision_Edge(p_pos, p_angle, EGO, gridConverted, param)
col = 0;
m = ceil(param.minDist/param.minRes);
t = linspace(0, param.minDist, m);
ex=cosd(p_angle);
ey=sind(p_angle);
t_x = t*ex;
t_y = t*ey;
exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);

for i = 2:(m-1)
    p_pos_step(1) = p_pos(1)+t_x(i);
    p_pos_step(2) = p_pos(2)+t_y(i);
    
    P(1)=p_pos_step(1)+ex*(EGO.lf)+EGO.w/2*exOrtho;
    P(2)=p_pos_step(2)+ey*(EGO.lf)+EGO.w/2*eyOrtho;
    P(3)=p_pos_step(1)+ex*(EGO.lf)-EGO.w/2*exOrtho;
    P(4)=p_pos_step(2)+ey*(EGO.lf)-EGO.w/2*eyOrtho;
    P(5)=p_pos_step(1)-ex*(EGO.lr)+EGO.w/2*exOrtho;
    P(6)=p_pos_step(2)-ey*(EGO.lr)+EGO.w/2*eyOrtho;
    P(7)=p_pos_step(1)-ex*(EGO.lr)-EGO.w/2*exOrtho;
    P(8)=p_pos_step(2)-ey*(EGO.lr)-EGO.w/2*eyOrtho;
    
    % Conversion to grid
    p_step(1) = round(p_pos_step(1)/0.5);
    p_step(2) = round(p_pos_step(2)/0.5);
    P_grid = round(P/(0.5));
    
    % Checking for occupancy value
    probOccupancy(1) = gridConverted(p_step(1), p_step(2), 1);
    probOccupancy(2) = gridConverted(P_grid(1), P_grid(2), 1);
    probOccupancy(3) = gridConverted(P_grid(3), P_grid(4), 1);
    probOccupancy(4) = gridConverted(P_grid(5), P_grid(6), 1);
    probOccupancy(5) = gridConverted(P_grid(7), P_grid(8), 1);
    
    if param.plotGraph
        figure(2);
        plotObject_2(1) = plot([P_grid(1) P_grid(3)], [P_grid(2) P_grid(4)], 'Color', 'b', 'LineWidth', 2);
        plotObject_2(2) = plot([P_grid(1) P_grid(5)], [P_grid(2) P_grid(6)], 'Color', 'b');
        plotObject_2(3) = plot([P_grid(3) P_grid(7)], [P_grid(4) P_grid(8)], 'Color', 'b');
        plotObject_2(4) = plot([P_grid(7) P_grid(5)], [P_grid(8) P_grid(6)], 'Color', 'b');
    end
    
    if any(probOccupancy > param.minThresh)
        col = 1;
        if param.plotGraph
            delete(plotObject_2);
        end
        return;
    end
end
end


