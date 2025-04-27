%--------------------------------------------------------------------------
%            Parthasarathy Nadarajan
%            CARISSMA & Technische Hochschule Ingolstadt (THI)
%            Path Planning using RRT and Predicted Occupancy Grids
%            Using three different acceleration profiles
%--------------------------------------------------------------------------
clc; clearvars; close all;

%% Define the configuration space
load('POG');

% Rotate the occupancy grid
% gridConverted = rot90(predictedOccupancyGrid, 3);
gridConverted = rot90(permute(POG, [2 3 1]), 3);
gridConverted(1,1,:) = 0;
% Plot the occupied cells
[a, b] = find(gridConverted(:,:,1) >0.0);
figure(1);
s1 = scatter(a, b, 'k', 's', 'filled'); hold on
axis([1 80 1 80])
figure(2);
s2 = scatter(a*0.5, b*0.5, 'k', 's', 'filled'); hold on
axis([1 40 1 40])
%% EGO vehicle data
EGO.xCG = 3; % m
EGO.yCG = 20; % m
EGO.vel = 9; % m/s
EGO.psi = 0; % degrees
EGO.ax = 0; % m/s^2
EGO.ay = 0; % m/s^2
EGO.lf = 2; % m
EGO.lr = 2; % m
EGO.w = 2; % m
EGO.steerMax = 4; % degrees

%% Parameters for RRT algorithm
param.RRTSampleTime = 0.1; % new node sampling interval
param.simulationSampleTime = 0.02; % simulation step
param.maxIter = 4000;
param.minThresh = 0.25; % minimum probability of occupancy
param.stopThresh = 3; % distance around the goal to stop simulation
param.plotGraph = false;
saveImage = true;

%% Define the start and goal point
% Prediction time
tPred = 2.75; % seconds

% Start point in terms of grid
p_start = [round(EGO.xCG/0.5), round(EGO.yCG/0.5)];
% Start point in terms of position
p_pos_start(1) = EGO.xCG;
p_pos_start(2) = EGO.yCG;

% EGO position after 2 seconds will be the goal position (without any change in the state)
distMoved = EGO.vel*tPred + 0.5*EGO.ax*tPred^2;
tx_pos = distMoved*cosd(EGO.psi);
ty_pos = distMoved*sind(EGO.psi);
p_pos_goal(1) = 25;
p_pos_goal(2) = 23;
% Goal point in terms of grid
p_goal = [round(p_pos_goal(1)/0.5), round(p_pos_goal(2)/0.5)];

% Plot the start and the goal point
figure(1)
scatter(p_start(1), p_start(2), 'r', 'filled', 'd');
scatter(p_goal(1), p_goal(2), 'b', 'filled', 'd');
legend('Occupied', 'Start', 'Goal');
legend('AutoUpdate', 'off')
figure(2);
scatter(p_pos_start(1), p_pos_start(2), [], [0.45 0.45 0.45], 'filled', 'd');
scatter(p_pos_goal(1), p_pos_goal(2), 'b', 'filled', 'd');
legend('Occupied', 'Start', 'Goal');
legend('AutoUpdate', 'off')

%% Initialise the tree
% RRT with constant velocity profile
rrtConstantVel = [];
rrtConstantVel = addNode(rrtConstantVel, p_start, p_pos_start, 0, EGO.psi, EGO.vel, 1, 0, 1); %0 - root node; 1 - simulationStep; 0 - min prob; 1 - terminal node;

% RRT with braking acceleration profile
rrtBraking = [];
rrtBraking = addNode(rrtBraking, p_start, p_pos_start, 0, EGO.psi, EGO.vel, 1, 0, 1); %0 - root node; 1 - simulationStep; 0 - min prob; 1 - terminal node;

% RRT with constant state profile
rrtConstantState = [];
rrtConstantState = addNode(rrtConstantState, p_start, p_pos_start, 0, EGO.psi, EGO.vel, 1, 0, 1); %0 - root node; 1 - simulationStep; 0 - min prob; 1 - terminal node;

%% Path planning
% Path planning with the constant state profile
tic
[rrtConstantState, iterConstantState] = pathPlanning(p_goal, p_pos_goal, gridConverted, rrtConstantState, EGO, param);
timeTakenCS = toc;

% Path planning with constant velocity profile
tic
EGO.ax = 0;
[rrtConstantVel, iterConstantVel] = pathPlanning(p_goal, p_pos_goal, gridConverted, rrtConstantVel, EGO, param);
timeTakenCV = toc;

% Path planning with braking profile
tic
EGO.ax = -2;
[rrtBraking, iterBraking] = pathPlanning(p_goal, p_pos_goal, gridConverted, rrtBraking, EGO, param);
timeTakenBr = toc;

%% Extracting the path from tree
% Constant state profile---------------------------------------------------
constantStatePath = extractPath(rrtConstantState, p_goal, 'y');

% Constant velocity profile------------------------------------------------
constantVelPath = extractPath(rrtConstantVel, p_goal, 'm');

% Braking profile----------------------------------------------------------
brakingPath = extractPath(rrtBraking, p_goal, 'c');

%% Plot the unsmoothed path
% Constant state profile---------------------------------------------------

if length(constantStatePath.sampleTime) < 20
    disp('Could not find valid path with constant state profile');
    maxProb(1) = Inf;
else
    maxProb(1) = max(constantStatePath.prob);
    fprintf('Time taken for the RRT algorithm to find a path using constant state profile is %d \n', timeTakenCS);
    fprintf('The number of iterations taken is %d \n', iterConstantState-1);
    saveDir = 'ConstantState';
    plotGraphWithOcc(constantStatePath, gridConverted, EGO, saveImage, saveDir, 'y')
    plotRRTNode(rrtConstantState, 'y')
end

% Constant Velocity profile------------------------------------------------
if length(constantVelPath.sampleTime) < 20
    disp('Could not find valid path with constant velocity profile');
    maxProb(2) = Inf;
else
    maxProb(2) = max(constantVelPath.prob);
    fprintf('Time taken for the RRT algorithm to find a path using constant velocity profile is %d \n', timeTakenCV);
    fprintf('The number of iterations taken is %d \n', iterConstantVel-1);
    saveDir = 'ConstantVelocity';
    plotGraphWithOcc(constantVelPath, gridConverted, EGO, saveImage, saveDir, 'm')
    plotRRTNode(rrtConstantVel, 'm')
end

% Braking profile----------------------------------------------------------
if length(brakingPath.sampleTime) < 20
    disp('Could not find valid path with braking profile');
    maxProb(3) = Inf;
else
    maxProb(3) = max(brakingPath.prob);
    fprintf('Time taken for the RRT algorithm to find a path using braking profile is %d \n', timeTakenBr);
    fprintf('The number of iterations taken is %d \n', iterBraking-1);
    saveDir = 'BrakeProfile';
    plotGraphWithOcc(brakingPath, gridConverted, EGO, saveImage, saveDir, 'c')
    plotRRTNode(rrtBraking, 'c')
end

%% Using the controller to follow the unsmoothed path
% Constant state profile
if length(constantStatePath.sampleTime) > 19
    controller_modified(constantStatePath.pos(1,:), constantStatePath.pos(2,:), EGO.psi*3.14/180, constantStatePath.vel(1), constantStatePath.vel(end), EGO.ax, EGO.ay, param)
end
% Constant Velocity profile------------------------------------------------
if length(constantVelPath.sampleTime) > 19
    controller_modified(constantVelPath.pos(1,:), constantVelPath.pos(2,:), EGO.psi*3.14/180, constantVelPath.vel(1), constantVelPath.vel(end), EGO.ax, EGO.ay, param)
end
% Braking profile----------------------------------------------------------
if length(brakingPath.sampleTime) > 19
    controller_modified(brakingPath.pos(1,:), brakingPath.pos(2,:), EGO.psi*3.14/180, brakingPath.vel(1), brakingPath.vel(end), EGO.ax, EGO.ay, param)
end

%% Choosing the safest path
[val, id] = min(maxProb);
if id == 1
    disp('Safest path is constant state profile');
elseif id == 2
    disp('Safest path is constant velocity profile');
elseif id == 3
    disp('Safest path is braking profile');
elseif val == Inf
    disp('Could not find path, trigger AEB !');
end

%% Additional functions
% Function - Add node to the tree------------------------------------------
function rrt = addNode(rrt, p, p_pos, iPrev, p_angle, p_vel, sampleTime, min_prob, termNode)
node.p = p;
node.p_pos = p_pos;
node.iPrev = iPrev;
node.angPrev = p_angle;
node.vel = p_vel;
node.sampleTime = sampleTime;
node.terminalNode = termNode;
node.prob = min_prob;
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
    if rem(iter, 10) == 0
        p_rand(1) = p_goal(1);
        p_rand(2) = p_goal(2);
    else
        p_rand(1) = randperm(79, 1);
        p_rand(2) = randperm(79, 1);
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
        plotObject(3) = scatter(p_pos_rand(1), p_pos_rand(2), 'g', 'filled');
        plotObject(4) = scatter(a*0.5, b*0.5, 'k', 's', 'filled');
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
        plotObject(5) = scatter(l(1), l(2), 'r*');
        figure(2)
        plotObject(6) = scatter(p_pos(1), p_pos(2), 'r*');
    end
    
    % Find a path from the selected nearest vertex to the chosen random vertex
    [col, p_step, p_pos_step, p_angle, p_vel, p_prob] = InCollision_Edge(p_pos, p_pos_rand, min_l_angle, min_p_angle, min_p_vel, min_angle_diff, gridConverted, EGO, param, sampleTime);
    
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
        figure(1);
        plotObject1(7) = scatter(p_step(1), p_step(2), 'k', 'filled');
        figure(2);
        plotObject1(8) = scatter(p_pos_step(1), p_pos_step(2), [], [1 165/255 0], 'filled');
    end
    
    % Increment the sample time by 1 when a valid vertex is found
    sampleTime = sampleTime+1;
    
    % Add the valid vertex to the tree
    rrt = addNode(rrt, p_step, p_pos_step, imin, p_angle, p_vel, sampleTime, p_prob, 1);
    
    % Calculating the distance to the goal
    dist = norm(p_pos_step-p_pos_goal);
    
    % Considering the distance to the goal
    if (dist <= param.stopThresh) || (iter >= param.maxIter) || (sampleTime == 20)
        break
    end
    iter = iter+1;
    if param.plotGraph
        delete(plotObject)
    end
end
end

% Function - Extracting the path from the RRT------------------------------
function Path = extractPath(rrt, p_goal, color)
i = length(rrt);
% Select the node that is closest to the goal as well as a terminal node
for j = 1:i
    dist = sqrt((rrt(j).node.p(1) -p_goal(1))^2 + (rrt(j).node.p(2)-p_goal(2))^2);
    if (j == 1) || (dist < mindist) && (rrt(j).node.terminalNode == 1) && (rrt(j).node.sampleTime == 20)
        mindist = dist;
        imin = j;
    end
end

% Plotting the final position
figure(1);
scatter(rrt(imin).node.p(1), rrt(imin).node.p(2), color, 'filled', 'd', 'LineWidth', 2);
figure(2);
scatter(rrt(imin).node.p_pos(1), rrt(imin).node.p_pos(2), color, 'filled', 'd', 'LineWidth', 2);

% Extracting the required information from the RRT
Path.grid = rrt(imin).node.p';
Path.pos = rrt(imin).node.p_pos';
Path.angle = rrt(imin).node.angPrev;
Path.sampleTime = rrt(imin).node.sampleTime;
Path.vel = rrt(imin).node.vel;
Path.prob = rrt(imin).node.prob;
j = imin;

while 1
    j = rrt(j).node.iPrev;
    if j == 0
        break
    end
    Path.grid = [rrt(j).node.p' Path.grid];
    Path.pos = [rrt(j).node.p_pos' Path.pos];
    Path.angle = [rrt(j).node.angPrev Path.angle];
    Path.sampleTime = [rrt(j).node.sampleTime Path.sampleTime];
    Path.vel = [rrt(j).node.vel Path.vel];
    Path.prob = [rrt(j).node.prob Path.prob];
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
function plotGraphWithOcc(Path, gridConverted, EGO, saveImage, saveDir, color)
for i = 2:length(Path.grid)
    figure(1);
    plot([Path.grid(1,i);Path.grid(1,i-1)],[Path.grid(2,i);Path.grid(2,i-1)], color, 'LineWidth',2);
    
    if saveImage
        figure(3);
        
        % Plot of the POG for the corresponding time instance
        [a, b] = find(gridConverted(:,:,Path.sampleTime(i)) >0.0);
        for j = 1:length(a)
            s2 = scatter(a(j)*0.5, b(j)*0.5, 36, gridConverted(a(j), b(j), Path.sampleTime(i)), 'filled', 's', 'LineWidth', 2); hold on
        end
        % Plot of the selected point
        scatter(Path.pos(1,i),Path.pos(2,i),'ko', 'filled');
    end
    figure(2);
    plot([Path.pos(1,i);Path.pos(1,i-1)],[Path.pos(2,i);Path.pos(2,i-1)], color ,'LineWidth',2);
    
    % Plotting the vehicle
    if saveImage
        ex=cosd(Path.angle(i));
        ey=sind(Path.angle(i));
        exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
        eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
        P_plot(1)=Path.pos(1,i)+ex*(EGO.lf)+EGO.w/2*exOrtho;
        P_plot(2)=Path.pos(2,i)+ey*(EGO.lf)+EGO.w/2*eyOrtho;
        P_plot(3)=Path.pos(1,i)+ex*(EGO.lf)-EGO.w/2*exOrtho;
        P_plot(4)=Path.pos(2,i)+ey*(EGO.lf)-EGO.w/2*eyOrtho;
        P_plot(5)=Path.pos(1,i)-ex*(EGO.lr)+EGO.w/2*exOrtho;
        P_plot(6)=Path.pos(2,i)-ey*(EGO.lr)+EGO.w/2*eyOrtho;
        P_plot(7)=Path.pos(1,i)-ex*(EGO.lr)-EGO.w/2*exOrtho;
        P_plot(8)=Path.pos(2,i)-ey*(EGO.lr)-EGO.w/2*eyOrtho;
        figure(3);
        %p(1) = plot([P_plot(1) P_plot(3)], [P_plot(2) P_plot(4)], 'Color', 'b', 'LineWidth', 2);
        %p(2) = plot([P_plot(1) P_plot(5)], [P_plot(2) P_plot(6)], 'Color', 'b');
        %p(3) = plot([P_plot(3) P_plot(7)], [P_plot(4) P_plot(8)], 'Color', 'b');
        %p(4) = plot([P_plot(7) P_plot(5)], [P_plot(8) P_plot(6)], 'Color', 'b');
        p = fill([P_plot(1) P_plot(3) P_plot(7) P_plot(5)], [P_plot(2) P_plot(4) P_plot(8) P_plot(6)], 'r');
        for j = 2:i
            plot([Path.pos(1,j-1) Path.pos(1,j)], [Path.pos(2,j-1) Path.pos(2,j)], 'b');
            scatter(Path.pos(1,j), Path.pos(2,j), 'filled', 'k');
        end
        axis([1 40 1 40])
        axis off
        mkdir(saveDir)
        h = figure(3);
        set(h,'Units','Inches');
        pos = get(h,'Position');
        set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
        saveName = num2str(i);
        fileName = fullfile(saveDir, saveName);
        print(gcf, fileName, '-dpdf', '-r600')
        delete(p);
        delete(s2);
        close(figure(3))
    end
end
end
