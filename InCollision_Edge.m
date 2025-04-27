% Function - Collision check along the edge--------------------------------
function [col, p_step, p_pos_step, p_angle, p_vel, p_prob] = InCollision_Edge(p_pos, ~, l_angle, p_angle, l_vel, angle_diff, gridConverted, EGO, param, sampleTime)

% Determining the change in the steering angle
if (p_angle >=0 && p_angle <=90 && l_angle >= 270 && l_angle <=360)
    if abs(angle_diff) > EGO.steerMax
        p_angle = l_angle+EGO.steerMax;
    end
elseif (l_angle >=0 && l_angle <=90 && p_angle >= 270 && p_angle <=360)
    if abs(angle_diff) > EGO.steerMax
        p_angle = l_angle-EGO.steerMax;
    end
else
    if abs(angle_diff) > EGO.steerMax && angle_diff < 0
        p_angle = l_angle+EGO.steerMax;
    elseif abs(angle_diff) > EGO.steerMax && angle_diff > 0
        p_angle = l_angle-EGO.steerMax;
    end
end
if p_angle < 0
    p_angle = 360+p_angle;
end

% Initialise variables
p_step = zeros(1,2);
p_pos_step = zeros(1,2);
P = zeros(8,1);
P_grid = zeros(8,1);
probOccupancy = zeros(5,1);
col = 0;

% Estimation of distance for every step of RRT
minRes = l_vel*param.simulationSampleTime + 0.5*EGO.ax*param.simulationSampleTime^2; % steps in between RRT
minDist = l_vel*param.RRTSampleTime + 0.5*EGO.ax*param.RRTSampleTime^2; % distance constrained based on RRT sample time
p_vel = l_vel+EGO.ax*param.RRTSampleTime; 
% To prevent negative velocities
if p_vel <=0 
    p_vel = 0;
end
m = ceil(minDist/minRes);
s = linspace(0, minDist, m);
ex=cosd(p_angle);
ey=sind(p_angle);
t_x = s*ex;
t_y = s*ey;
exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);

for i = 2:m
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
    probOccupancy(1) = gridConverted(p_step(1), p_step(2), sampleTime);
    probOccupancy(2) = gridConverted(P_grid(1), P_grid(2), sampleTime);
    probOccupancy(3) = gridConverted(P_grid(3), P_grid(4), sampleTime);
    probOccupancy(4) = gridConverted(P_grid(5), P_grid(6), sampleTime);
    probOccupancy(5) = gridConverted(P_grid(7), P_grid(8), sampleTime);  
    
    if any(probOccupancy > 0)
        p_prob = max(probOccupancy);
    if any(probOccupancy >= param.minThresh)
        col = 1;
        return;
    end
    else
        p_prob = 0;
    end
end
end
