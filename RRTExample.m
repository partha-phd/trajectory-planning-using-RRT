%--------------------------------------------------------------------------
%            Parthasarathy Nadarajan
%            CARISSMA & Technische Hochschule Ingolstadt (THI)
%            Path Planning using RRT and Predicted Occupancy Grids
%--------------------------------------------------------------------------
clc; clearvars; close all;

%% Define the configuration space
load('POG_scenario');

% Rotate the occupancy grid
gridConverted = rot90(predictedOccupancyGrid, 3);

% Plot the occupied cells
[a, b] = find(gridConverted(:,:,1) >0.01);
figure(2);
axis([1 80 1 80])
s1 = scatter(a, b, 'k', 's', 'filled'); hold on
figure(3);
axis([1 80 1 80])
s2 = scatter(a*0.5, b*0.5, 'k', 's', 'filled'); hold on

%% EGO vehicle data
EGO.vel = 5; % m/s
EGO.psi = 350; % degrees
EGO.ax = 0; % m/s^2
EGO.ay = 0; % m/s^2
EGO.xCG = 10; % m
EGO.yCG = 20; % m
EGO.lf = 2; % m
EGO.lr = 2; % m
EGO.w = 2; % m
EGO.steerMax = 1; % degrees

%% Define the start and goal point
p_start = [EGO.xCG/0.5, EGO.yCG/0.5]; % grid position
% Start point in terms of position
p_pos_start(1) = EGO.xCG;
p_pos_start(2) = EGO.yCG;

% EGO position after 2 seconds will be the goal position
distMoved = EGO.vel*2;
tx_pos = distMoved*cosd(EGO.psi);
ty_pos = distMoved*sind(EGO.psi);
p_pos_goal(1) = EGO.xCG+tx_pos;
p_pos_goal(2) = EGO.yCG+ty_pos;

p_goal = [round(p_pos_goal(1)/0.5), round(p_pos_goal(2)/0.5)]; % grid position

% Plot the start and the goal point
figure(2)
scatter(p_start(1), p_start(2), 'r', 'filled', 'd');
scatter(p_goal(1), p_goal(2), 'b', 'filled', 'd');
legend('Occupied', 'Start', 'Goal');
legend('AutoUpdate', 'off')
figure(3);
scatter(p_pos_start(1), p_pos_start(2), 'r', 'filled', 'd');
scatter(p_pos_goal(1), p_pos_goal(2), 'b', 'filled', 'd');
legend('Occupied', 'Start', 'Goal');
legend('AutoUpdate', 'off')

%% Parameters for RRT algorithm
param.maxIter = 2000;
param.minThresh = 0.1; % minimum probability of occupancy
RRTSampleTime = 0.1;
simulationSampleTime = 0.02;
param.minRes = EGO.vel*simulationSampleTime; % steps in between RRT
param.minDist = EGO.vel*RRTSampleTime; % distance constrained based on RRT sample time
param.stopThresh = 0.5; % distance around the goal to stop simulation
param.plotGraph = false;
param.minAngle = 10; % constraints on the steering

%% Initialise the tree
rrt = {};
rrt = AddNode(rrt, p_start, p_pos_start, 0, EGO.psi, 1, 1); %0 - root node, %1 - simulationTime is 1 % 1 - terminal node

%% Path planning
tic
[rrt, iter] = pathPlanning(p_goal, p_pos_goal, gridConverted, rrt, EGO, param);
timeTaken = toc;

%% Extracting the path from tree
i = length(rrt);
% Select the node that is closest to the goal as well as a terminal node
for j = 1:i
    dist = sqrt((rrt{j}.p(1) -p_goal(1))^2 + (rrt{j}.p(2)-p_goal(2))^2);
    if (j == 1) || (dist < mindist) && (rrt{j}.terminalNode == 1) && (rrt{j}.sampleTime == 21)
        mindist = dist;
        imin = j;
    end
end

figure(2);
scatter(rrt{imin}.p(1), rrt{imin}.p(2), 'g', 'filled', 'd', 'LineWidth', 2);
P = rrt{imin}.p';
P_pos = rrt{imin}.p_pos';
P_angle = rrt{imin}.angPrev;
j = imin;
while 1
    j = rrt{j}.iPrev;
    if j == 0
        break
    end
    P = [rrt{j}.p' P];
    P_pos = [rrt{j}.p_pos' P_pos];
    P_angle = [rrt{j}.angPrev P_angle];
end

%% Plot the unsmoothed path
if isempty(P)
    disp('Could not find valid path');
else
    fprintf('Time taken for the RRT algorithm to find a path is %d \n', timeTaken);
    fprintf('The number of iterations taken is %d \n', iter-1);
    
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
    if param.plotGraph
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
end
%% Controller for following the path
tic
utils.controller_modified(P_pos(1,:), P_pos(2,:), EGO.psi*3.14/180, EGO.vel, EGO.vel, EGO.ax, EGO.ay, param)
toc
%% Additional functions
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
        % Find the occupied cells
        [a, b] = find(gridConverted(:,:,sampleTime) >0.01);
        figure(2)
        plotObject(1) = scatter(p_rand(1), p_rand(2), 'ro');
        plotObject(2) = scatter(a, b, 'g*');
        figure(3)
        plotObject(3) = scatter(p_pos_rand(1), p_pos_rand(2), 'ro');
        plotObject(4) = scatter(a*0.5, b*0.5, 'g*');
    end
    
    
    % Select the nearest vertex to the chosen random vertex
    for i = 1:length(rrt)
        
        % Heuristic 1 - Calculate the distance between the vertices
        dist =sqrt((rrt{i}.p(1) -p_rand(1))^2 + (rrt{i}.p(2)-p_rand(2))^2);
        % Heuristic 2 - Angle deviation required
        p_angle = atan2d(p_pos_rand(2)-rrt{i}.p(2)*0.5,...
            p_pos_rand(1)-rrt{i}.p(1)*0.5)+360*(p_pos_rand(2)-rrt{i}.p(2)*0.5<0);
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
            l_pos = rrt{i}.p_pos;
            l_angle = rrt{i}.angPrev;
            sampleTime = rrt{i}.sampleTime;
        end
    end
    
    % Convert grid to position
    p_pos(2) = l_pos(2);
    p_pos(1) = l_pos(1);
    
    % Plot the selected nearest vertex
    if param.plotGraph
        figure(2)
        plotObject(5) = scatter(l(1), l(2), 'b*');
        figure(3)
        plotObject(6) = scatter(p_pos(1), p_pos(2), 'b*');
    end
    
    % Find a path from the selected nearest vertex to chosen random vertex
    [col, p_step, p_pos_step, p_angle] = utils.InCollision_Edge(p_pos,...
        p_pos_rand, l_angle, gridConverted, EGO, param, sampleTime);
    
    % Skip to next iteration if it is not valid edge
    if col == 1
        iter = iter+1;
        if param.plotGraph
            delete(plotObject);
        end
        continue
    end
    if abs(l_angle-p_angle) > 1
        disp('blah');
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
    rrt = AddNode(rrt, p_step, p_pos_step, imin, p_angle, sampleTime, 1);
    
    % Calculating the distance to the goal
    dist = norm(p_pos_step-p_pos_goal);
    
    % Considering the distance to the goal
    if (dist <= param.stopThresh) || (iter >= param.maxIter) ||...
            (sampleTime == 21)
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
node.p = p;
node.p_pos = p_pos;
node.iPrev = iPrev;
node.angPrev = p_angle;
node.sampleTime = sampleTime;
node.terminalNode = termNode;
if length(rrt) >= 1
    rrt{1,iPrev}.terminalNode = 0;
end
rrt{end+1} = node;
end

% Function - Collision check along the edge--------------------------------
function [col, p_step, p_pos_step, p_angle] = InCollision_Edge(p_pos,...
    p_pos_rand, l_angle, gridConverted, EGO, param, sampleTime)
p_angle = atan2d(p_pos_rand(2)-p_pos(2), p_pos_rand(1)-p_pos(1))+...
    360*(p_pos_rand(2)-p_pos(2)<0);
% Checking for the change in the steering deviation
if (p_angle >=0 && p_angle <=90 && l_angle >= 270 && l_angle <=360)
    angle_diff = p_angle + (360-l_angle);
    if abs(angle_diff) > EGO.steerMax
        p_angle = l_angle+EGO.steerMax;
    end
elseif (l_angle >=0 && l_angle <=90 && p_angle >= 270 && p_angle <=360)
    angle_diff = l_angle + (360-p_angle);
    if abs(angle_diff) > EGO.steerMax
        p_angle = l_angle-EGO.steerMax;
    end
else
    angle_diff = l_angle-p_angle;
    if abs(angle_diff) > EGO.steerMax && angle_diff < 0
        p_angle = l_angle+EGO.steerMax;
    elseif abs(angle_diff) > EGO.steerMax && angle_diff > 0
        p_angle = l_angle-EGO.steerMax;
    end
end
if p_angle < 0
    p_angle = 360+p_angle;
end
col = 0;
m = ceil(param.minDist/param.minRes);
t = linspace(0, param.minDist, m);
ex=cosd(p_angle);
ey=sind(p_angle);
t_x = t*ex;
t_y = t*ey;
exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);

for i = 2:m
    p_pos_step(1)  = p_pos(1)+t_x(i);
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
    probOccupancy(1) = gridConverted(p_step(1), p_step(2), sampleTime);
    probOccupancy(2) = gridConverted(P_grid(1), P_grid(2), sampleTime);
    probOccupancy(3) = gridConverted(P_grid(3), P_grid(4), sampleTime);
    probOccupancy(4) = gridConverted(P_grid(5), P_grid(6), sampleTime);
    probOccupancy(5) = gridConverted(P_grid(7), P_grid(8), sampleTime);
    
    if param.plotGraph
        figure(3);
        plotObject_2(1) = plot([P(1) P(3)], [P(2) P(4)], 'Color', 'b',...
            'LineWidth', 2);
        plotObject_2(2) = plot([P(1) P(5)], [P(2) P(6)], 'Color', 'b');
        plotObject_2(3) = plot([P(3) P(7)], [P(4) P(8)], 'Color', 'b');
        plotObject_2(4) = plot([P(7) P(5)], [P(8) P(6)], 'Color', 'b');
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



