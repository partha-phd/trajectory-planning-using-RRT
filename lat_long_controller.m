%--------------------------------------------------------------------------
%    Trajectory planning in three stages: 1. path planning with linear segments,  
%    2. longitudinal dynamics on the path and 3. use of a vehicle dynamics
%    model to follow the first trajectory and generate in this way
%                    a drivable reference trajectory
%                    
%                           Michael Botsch
%--------------------------------------------------------------------------
clear all; clc; close all;


tic

% Conditions fpr trajectory
% rXpoints=[0 2 130 135]; % X-position of points in inertial frame E
% rYpoints=[0 0 2  2 ]; % Y-position of points in inertial frame E
% rXpoints=[0 2 45 50]; % X-position of points in inertial frame E
% rYpoints=[0 0 2  2 ]; % Y-position of points in inertial frame E
rXpoints=[0 2 25 30 35 55  60 65 95 100]; % X-position of points in inertial frame E
rYpoints=[0 0 2  2  2  -2  -2 -2 0  0]; % Y-position of points in inertial frame E
nrSections=size(rXpoints, 2)-1;
Psit0=0*pi/180; % Initial yas angle of EGO
vt0=0/3.6; % EGO velocity (magnitude of velocity vector) in vehicle frame F at t0
vt1=60/3.6; % EGO velocity (magnitude of velocity vector) in vehicle frame F at t1
axt0=0; % EGO x-aceleration in vehicle frame F at t0
ayt0=0; % EGO y-aceleration in vehicle frame F at t0

% Start time
t0=0;

% Desired time to reach goal
%t1=4;
t1=norm([rXpoints(end); rYpoints(end)]-[rXpoints(1); rYpoints(1)])/mean([vt1; vt0]);

% total time for trajectory
deltat1=t1-t0;

Ttraj=0.02; % discretization time for trajectory computation
tTraj=t0:Ttraj:t1; % time for trajectory
Ntraj=length(tTraj);

% Velocity and Time for path 1, which is used only for planning 
vPath=20/3.6;
deltaPath=nan(nrSections+1, 1);
Nsegment=nan(nrSections+1, 1);
for idxCurrentSection=2:nrSections+1
    deltaPath(idxCurrentSection)=norm([rXpoints(idxCurrentSection); rYpoints(idxCurrentSection)]-...
        [rXpoints(idxCurrentSection-1); rYpoints(idxCurrentSection-1)])/vPath;
    Nsegment(idxCurrentSection)=floor((deltaPath(idxCurrentSection))/Ttraj)+1;
end
deltaPath=deltaPath(2:end);
Nsegment=Nsegment(2:end);
NPath=sum(Nsegment);


% Initialization
rXPath=nan(Ntraj,1); % X-position in inertial frame E for path
rXtraj=nan(Ntraj,1); % X-position in inertial frame E for trajectory
rXtraj(1)=rXpoints(1);
rXtraj(end)=rXpoints(end);
vXtraj=nan(Ntraj,1); % velocity in X-direction in inertial frame E
aXtraj=nan(Ntraj,1); % acceleration in X-direction in inertial frame E
rYPath=nan(Ntraj,1); % Y-position in inertial frame E for path
rYtraj=nan(Ntraj,1); % Y-position in inertial frame E for trajectory
rYtraj(1)=rYpoints(1);
rYtraj(end)=rYpoints(end);
vYtraj=nan(Ntraj,1); % velocity in Y-direction in inertial frame E
aYtraj=nan(Ntraj,1); % acceleration in Y-direction in inertial frame E
PsiTraj=nan(Ntraj,1); % yaw angele in inertial frame E
PsiTraj(1)=Psit0;
vTraj=nan(Ntraj,1); % magnitude of the velocity
vTraj(1)=vt0;
vTraj(end)=vt1;
axTraj=nan(Ntraj,1); % longitudinal acceleration in vehicle frame F
axTraj(1)=axt0;
ayTraj=nan(Ntraj,1); % lateral acceleration in vehicle frame F
ayTraj(1)=ayt0;


%--------------------------------------------------------------------------
% FIRST STAGE: COMPUTATION OF PATH WITH LINEAR SEGMENTS THAT PASS THROUGH
% THE GIVEN POINTS
%--------------------------------------------------------------------------

dPfad=zeros(Ntraj,1);

nCurrentSegment=0;
for idxCurrentSection=1:nrSections
    % Matrix for polynomial approach
    A=[0 1; deltaPath(idxCurrentSection) 1];
    % Initial conditions for X-direction
    lX=[rXpoints(idxCurrentSection); rXpoints(idxCurrentSection+1)];
    % Coefficients of 1-st order polynom for X-direction of inertial frame E
    hX=A\lX;
    % Initial conditions for X-direction
    lY=[rYpoints(idxCurrentSection); rYpoints(idxCurrentSection+1)];
    % Coefficients of 1-st order polynom for the path
    hY=A\lY;
    
    % Computation of segments 1 
    for n=nCurrentSegment+1:floor((sum(Nsegment(1:idxCurrentSection))/NPath)*Ntraj)
        rXPath(n)=hX(1)*(NPath/Ntraj*(tTraj(n)-tTraj(nCurrentSegment+1)))+ hX(2);
        rYPath(n)=hY(1)*(NPath/Ntraj*(tTraj(n)-tTraj(nCurrentSegment+1))) + hY(2);
        if n>1
            dPfad(n)=dPfad(n-1)+sqrt((rXPath(n)-rXPath(n-1))^2 + (rYPath(n)-rYPath(n-1))^2);
        end
    end
    nCurrentSegment=n;
end


%--------------------------------------------------------------------------
% SECOND STAGE: COMPUTATION OF VELOCITY FOR PATH ACCORDING TO THE REQUIREMENTS 
% FOR THE INITIAL AND FINAL VELOCITY
%--------------------------------------------------------------------------

% two possibilities are implemented: polynmial approach of the velocity on the patho and piecewise constant acceleration profile

% Matrix for polynomial approach for longitudinal dynamics on path
A=[0 0 0 1; deltat1^3 deltat1^2 deltat1 1; 0 0 1 0; 3*deltat1^2 2*deltat1 1 0];
% Initial conditions for X-direction
ld=[0; dPfad(end); vt0; vt1];
% Coefficients of 3-rd order polynomial for longitudinal dynamics on the path
hd=A\ld;

% Values for piecewise constant acceleration profile:
tStartSegment=t1-2*(dPfad(end)-vt0*t1)/(vt1-vt0); % piecewiese constant for acceleration: time where the segmet starts
aSegment=(vt1-vt0)/(t1-tStartSegment); % piecewiese constant segment for acceleration: acceleration
pathLenthCurrent=0; % length of path

for n=1:Ntraj
    if (abs(vt1-vt0)<1) || (tStartSegment<0) || (tStartSegment>t1) || (abs(aSegment)>10) % conditions where piecewise constant acceleration profile not suited
        % POLYNOMIAL APPROACH FOR VELOCITY
        vTraj(n)=3*hd(1)*(tTraj(n)-t0)^2 + 2*hd(2)*(tTraj(n)-t0) + hd(3); % current velocity on the path
        pathLenthCurrent=hd(1)*(tTraj(n)-t0)^3 + hd(2)*(tTraj(n)-t0)^2 + hd(3)*(tTraj(n)-t0)+hd(4); % current length of path
        % Associate position points on path with the position points of final trajectory (dependent on the computed velocity)
        [minVal, minIdx]=min(abs(dPfad-pathLenthCurrent));
        rXtraj(n)=rXPath(minIdx);
        rYtraj(n)=rYPath(minIdx);
        if n>1
            if (abs(rXtraj(n)-rXtraj(n-1))<1e-6) && (abs(rYtraj(n)-rYtraj(n-1))<1e-6) % same point on the trajectory from the path
                PsiTraj(n)=PsiTraj(n-1); % current yaw angle
                axTraj(n)=axTraj(n-1); % current longituinal acceleration
                ayTraj(n)=ayTraj(n-1); % current lateral acceleration
            else
                PsiTraj(n)=atan2(rYtraj(n)-rYtraj(n-1),rXtraj(n)-rXtraj(n-1)); % current yaw angle
                dPsiTraj=(PsiTraj(n)-PsiTraj(n-1))/Ttraj; % current yaw rate
                axTraj(n)=(vTraj(n)-vTraj(n-1))/Ttraj; % current longituinal acceleration
                ayTraj(n)=vTraj(n)*dPsiTraj; % current lateral acceleration
            end
        end
        vXtraj(n)=vTraj(n)*cos(PsiTraj(n)); % current X-velocity in inertial frame E
        vYtraj(n)=vTraj(n)*sin(PsiTraj(n)); % current Y-velocity in inertial frame E
    else
        % PIECEWISE CONSTANT ACCELERATION PROFILE
        if (tTraj(n)-t0)<tStartSegment
            axTraj(n)=0;
        else
            axTraj(n)=aSegment;
        end
        %           % Alternatives for piecewise constant acceleratin profile:
        %             axTraj(n)=2*(dPfad-vt0*t1)/(t1^2);
        %             axTraj(n)=(vt1-vt0)/(t1);
        if n>1
            vTraj(n)=vTraj(n-1)+axTraj(n)*Ttraj; % current velocity on the path
            pathLenthCurrent=pathLenthCurrent+vTraj(n)*Ttraj+0.5*axTraj(n)*Ttraj^2; % current length of path
            % Associate position points on path with the position points of final trajectory (dependent on the computed velocity)
            [minVal, minIdx]=min(abs(dPfad-pathLenthCurrent));
            rXtraj(n)=rXPath(minIdx);
            rYtraj(n)=rYPath(minIdx);
            if (abs(rXtraj(n)-rXtraj(n-1))<1e-6) && (abs(rYtraj(n)-rYtraj(n-1))<1e-6) % same point on the trajectory from the path
                PsiTraj(n)=PsiTraj(n-1); % current yaw angle
                axTraj(n)=axTraj(n-1); % current longituinal acceleration
                ayTraj(n)=ayTraj(n-1); % current lateral acceleration
            else
                PsiTraj(n)=atan2(rYtraj(n)-rYtraj(n-1),rXtraj(n)-rXtraj(n-1)); % current yaw angle
                dPsiTraj=(PsiTraj(n)-PsiTraj(n-1))/Ttraj; % current yaw rate
                axTraj(n)=(vTraj(n)-vTraj(n-1))/Ttraj; % current longituinal acceleration
                ayTraj(n)=vTraj(n)*dPsiTraj; % current lateral acceleration
            end
        end
        vXtraj(n)=vTraj(n)*cos(PsiTraj(n)); % current X-velocity in inertial frame E
        vYtraj(n)=vTraj(n)*sin(PsiTraj(n)); % current Y-velocity in inertial frame E
    end
end



%--------------------------------------------------
% VIZUALIZATION OF FIRST (NOT DRIVABLE) TRAJECTORY
%--------------------------------------------------
%
    % Visualization of quantities of EGO as functions of time
    figure;
    subplot(6,1,1);
    h1=plot(tTraj, axTraj, 'r', 'LineWidth', 2);
    %leg1=legend(h1,{'$bla$'}); set(leg1,'Interpreter','latex');
    %legend('Location','northeast');
%     xticks([0 1 2 3]); %xticklabels({'t = 0','t = 2','t = 3'})
%     yticks([-ceil(max(abs(min(axTraj)),max(axTraj)))-0.1 0 ceil(max(abs(min(axTraj)),max(axTraj)))+0.1]); %yticklabels({'bla','bla'})
    axis([0 tTraj(end) -ceil(max(abs(min(axTraj)),max(axTraj)))-0.1 ceil(max(abs(min(axTraj)),max(axTraj)))+0.1]);
    xlabel('$t$ in s', 'Interpreter','latex');
    ylabel('${}^{F}_{E}\!a_{S,x}$ in $\frac{\mathrm{m}}{\mathrm{s}^2}$','Interpreter','latex');
    currentAxis=gca;
    set(currentAxis, 'FontName', 'Times New Roman');
    set(currentAxis, 'FontAngle', 'normal'); % 'italic'
    set(currentAxis, 'FontSize', 15);
    
    subplot(6,1,2);
    h2=plot(tTraj, ayTraj, 'b', 'LineWidth', 2);
    %leg2=legend(h2,{'$bla$'}); set(leg2,'Interpreter','latex');
    %legend('Location','northeast');
%     xticks([0 1 2 3]); %xticklabels({'t = 0','t = 2','t = 3'})
%     yticks([-ceil(max(abs(min(ayTraj)),max(ayTraj)))-0.1 0 ceil(max(abs(min(ayTraj)),max(ayTraj)))+0.1]); %yticklabels({'bla','bla'})
    axis([0 tTraj(end) -ceil(max(abs(min(ayTraj)),max(ayTraj)))-0.1 ceil(max(abs(min(ayTraj)),max(ayTraj)))+0.1]);
    xlabel('$t$ in s', 'Interpreter','latex');
    ylabel('${}^{F}_{E}\!a_{S,y}$ in $\frac{\mathrm{m}}{\mathrm{s}^2}$','Interpreter','latex');
    currentAxis=gca;
    set(currentAxis, 'FontName', 'Times New Roman');
    set(currentAxis, 'FontAngle', 'normal'); % 'italic'
    set(currentAxis, 'FontSize', 15);
    
    subplot(6,1,3);
    PsiGradPfad=PsiTraj*180/pi;
    h3=plot(tTraj, PsiGradPfad, 'b', 'LineWidth', 2);
    %leg3=legend(h3,{'$bla$'}); set(leg3,'Interpreter','latex');
    %legend('Location','northeast');
%     xticks([0 1 2 3]); %xticklabels({'t = 0','t = 2','t = 3'})
%     yticks([-ceil(max(abs(min(PsiGradPfad)),max(PsiGradPfad)))-0.1 0 ceil(max(abs(min(PsiGradPfad)),max(PsiGradPfad)))+0.1]); %yticklabels({'bla','bla'})
    axis([0 tTraj(end) -ceil(max(abs(min(PsiGradPfad)),max(PsiGradPfad)))-0.1 ceil(max(abs(min(PsiGradPfad)),max(PsiGradPfad)))+0.1]);
    xlabel('$t$ in s', 'Interpreter','latex');
    ylabel('$\Psi$ in $^\circ$','Interpreter','latex');
    currentAxis=gca;
    set(currentAxis, 'FontName', 'Times New Roman');
    set(currentAxis, 'FontAngle', 'normal'); % 'italic'
    set(currentAxis, 'FontSize', 15);
    %
    subplot(6,1,4);
    vMagnitudeKMHtraj=vTraj*3.6;
    h4=plot(tTraj,vMagnitudeKMHtraj, 'g', 'LineWidth', 2);
    %leg4=legend(h4,{'$bla$'}); set(leg4,'Interpreter','latex');
    %legend('Location','northeast');
%     xticks([0 1 2 3]); %xticklabels({'t = 0','t = 2','t = 3'})
%     yticks([0 ceil(max(abs(min(vMagnitudeKMHtraj)),max(vMagnitudeKMHtraj))/2) ceil(max(abs(min(vMagnitudeKMHtraj)),max(vMagnitudeKMHtraj)))]); %yticklabels({'bla','bla'})
    axis([0 tTraj(end) 0 ceil(max(abs(min(vMagnitudeKMHtraj)),max(vMagnitudeKMHtraj)))]);
    xlabel('$t$ in s', 'Interpreter','latex');
    ylabel('$v$ in $\frac{\mathrm{km}}{\mathrm{h}}$','Interpreter','latex');
    currentAxis=gca;
    set(currentAxis, 'FontName', 'Times New Roman');
    set(currentAxis, 'FontAngle', 'normal'); % 'italic'
    set(currentAxis, 'FontSize', 15);
    %
    subplot(6,1,5);
    h5=plot(tTraj, rXtraj, 'k', 'LineWidth', 2);
    %leg5=legend(h5,{'$bla$'}); set(leg5,'Interpreter','latex');
    %legend('Location','northeast');
%     xticks([0 1 2 3]); %xticklabels({'t = 0','t = 2','t = 3'})
%     yticks([floor(min(rXtraj)) round(0.5*(min(rXtraj)+max(rXtraj))) ceil(max(rXtraj))]);  %yticklabels({'bla','bla'})
    axis([0 tTraj(end) floor(min(rXtraj)) ceil(max(rXtraj))]);
    xlabel('$t$ in s', 'Interpreter','latex');
    ylabel('$r_X$ in m','Interpreter','latex');
    currentAxis=gca;
    set(currentAxis, 'FontName', 'Times New Roman');
    set(currentAxis, 'FontAngle', 'normal'); % 'italic'
    set(currentAxis, 'FontSize', 15);
    %
    subplot(6,1,6);
    h6=plot(tTraj, rYtraj, 'k', 'LineWidth', 2);
    %leg5=legend(h5,{'$bla$'}); set(leg5,'Interpreter','latex');
    %legend('Location','northeast');
%     xticks([0 1 2 3]); %xticklabels({'t = 0','t = 2','t = 3'})
%     yticks([floor(min(rYtraj))-0.1 round(0.5*(min(rYtraj)+max(rYtraj))) ceil(max(rYtraj))+0.1]);  %yticklabels({'bla','bla'})
    axis([0 tTraj(end) floor(min(rYtraj))-0.1 ceil(max(rYtraj))+0.1]);
    xlabel('$t$ in s', 'Interpreter','latex');
    ylabel('$r_Y$ in m','Interpreter','latex');
    currentAxis=gca;
    set(currentAxis, 'FontName', 'Times New Roman');
    set(currentAxis, 'FontAngle', 'normal'); % 'italic'
    set(currentAxis, 'FontSize', 15);
    %
    currentFigure=gcf;
    figName=sprintf('Zeitlicher Verlauf der dynamischen EGO-Größen der Solltrajektorie');
    set(currentFigure, 'Name', figName);
    scrsz = get(0,'ScreenSize');
    set(currentFigure, 'Position', [scrsz(4)*1/20 scrsz(4)*1/20 scrsz(4)*11/9 scrsz(4)*8/9]);
    %--------------------------------------------------------------------------
    % Save as PDF
    scrsz= get(0,'ScreenSize');
    set(gcf, 'PaperUnits', 'points');
    set(gcf, 'PaperSize', [scrsz(4)*9/9 scrsz(4)*6/9]);%[scrsz(4)*6/9 scrsz(4)*6/9]); %
    set(gcf, 'PaperPositionMode', 'manual');
    set(gcf, 'PaperPosition', [0 0 scrsz(4)*9/9 scrsz(4)*6/9]);%[0 0 scrsz(4)*6/9 scrsz(4)*6/9]); %
    set(gcf, 'renderer', 'painters');
    print(gcf, '-dpdf', 'FirstTrajectoryMultPointsStates3Stages.pdf');
    
    
    % Visualization of reference trajectory in inertial frame E
    w=1.8; % with of EGO
    ell=2.8; % wheelbase EGO
    lv=1.3; % distance from center of gravity to front axle of EGO
    lh=ell-lv; % distance from center of gravity to front axle of EGO
    lvToFront=0.85; % distance from front axle of EGO to front of car
    lhToRear=1.1; % distance from rear axle of EGO to rear of car
    
    figure;
    h7=plot(rXtraj, rYtraj, 'k', 'LineWidth', 2); % Plot trajectory of EGO
    %leg7=legend(h7,{'$r_Y$ vs. $r_X$'}); set(leg7,'Interpreter','latex');
    %legend('Location','northeast');
    %xticks([0 10 20 30 40]); %xticklabels({'rX=0','rX=10','rX=20','rX=30','rX=40'})
    %yticks([0 2 4 6 8]); %yticklabels({'r_Y=0','r_Y=2','r_Y=4','r_Y=6','r_Y=8'})
    %axis([0 40 0 8]);
    xlabel('$r_X$ in m', 'Interpreter','latex');
    ylabel('$r_Y$ in m','Interpreter','latex');
    hold on;
    %
    countPlotBodyShell=0;
    oldDistance=0;
    carStopped1=0;
    plotDistance=18;
    xthBodyShellWithText=1;
    for n=1:Ntraj
        % CORNERS OF EGO-CAR:
        ex=cos(PsiTraj(n));
        ey=sin(PsiTraj(n));
        exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
        eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
        P_vl_X=rXtraj(n)+ex*(lv+lvToFront)+w/2*exOrtho; % X-coordinate in E of the front left point of the car
        P_vl_Y=rYtraj(n)+ey*(lv+lvToFront)+w/2*eyOrtho; % Y-coordinate in E of the front left point of the car
        P_vr_X=rXtraj(n)+ex*(lv+lvToFront)-w/2*exOrtho; % X-coordinate in E of the front right point of the car
        P_vr_Y=rYtraj(n)+ey*(lv+lvToFront)-w/2*eyOrtho; % Y-coordinate in E of the front right point of the car
        P_rl_X=rXtraj(n)-ex*(lh+lhToRear)+w/2*exOrtho; % X-coordinate in E of the rear left point of the car
        P_rl_Y=rYtraj(n)-ey*(lh+lhToRear)+w/2*eyOrtho; % Y-coordinate in E of the rear left point of the car
        P_rr_X=rXtraj(n)-ex*(lh+lhToRear)-w/2*exOrtho; % X-coordinate in E of the rear right point of the car
        P_rr_Y=rYtraj(n)-ey*(lh+lhToRear)-w/2*eyOrtho; % Y-coordinate in E of the rear right point of the car
        %
        % VELOCITY VECTOR OF EGO-CAR
        exVel=cos(PsiTraj(n));
        eyVel=sin(PsiTraj(n));
        %
        %
        % VISUALIZE ONLY SOME OF THE COMPUTED VALUES:
        if (n>1) && (vTraj(n)==0 && vTraj(n-1)~=0)
            carStopped1=1;
        end
        if n>1
            oldDistance=oldDistance+sqrt((rXtraj(n)-rXtraj(n-1))^2+(rYtraj(n)-rYtraj(n-1))^2);
        end
        if (n==1) || (oldDistance > plotDistance) || (carStopped1==1) || (n==Ntraj) % tunig parameter
            oldDistance=0;
            %PLOT EGO
            plot([P_vl_X P_vr_X], [P_vl_Y P_vr_Y], 'b', 'LineWidth', 4);hold on; % plot front line of car
            plot([P_vl_X P_rl_X], [P_vl_Y P_rl_Y], 'b', 'LineWidth', 2);hold on; % plot left line of car
            plot([P_vr_X P_rr_X], [P_vr_Y P_rr_Y], 'b', 'LineWidth', 2);hold on; % plot right line of car
            plot([P_rr_X P_rl_X], [P_rr_Y P_rl_Y], 'b', 'LineWidth', 2);hold on; % plot rear line of car
            plot([rXtraj(n) rXtraj(n)+exVel*vTraj(n)],[rYtraj(n) rYtraj(n)+eyVel*vTraj(n)], 'g'); hold on; % plot velocity vector in green
            plot(rXtraj(n),rYtraj(n), 's',  'MarkerEdgeColor','k', 'MarkerFaceColor','b', 'MarkerSize',6); % plot center of gravity with square marker
            % SET TEXT ONLY TO SOME OF THE CAR VISUALIZATIONS:
            countPlotBodyShell=countPlotBodyShell+1;
            if (rem(countPlotBodyShell,xthBodyShellWithText)==0) || (n==1) || (carStopped1==1) || (n==N)
                % Text on EGO
                text(P_rl_X-5*exOrtho, P_rl_Y-5*eyOrtho, sprintf('$t=%2.1f$s',tTraj(n)), 'Rotation', PsiTraj(n)*180/pi, 'FontSize', 17, 'Interpreter','latex');
                text(P_rl_X-3*exOrtho, P_rl_Y-3*eyOrtho, sprintf('$v=%2.0f \\frac{\\mathrm{km}}{\\mathrm{h}}$',3.6*vTraj(n)), 'Rotation', PsiTraj(n)*180/pi, 'FontSize', 17, 'Interpreter','latex');
            end
        end
    end
    currentAxis=gca;
    set(currentAxis, 'FontName', 'Times New Roman');
    set(currentAxis, 'FontAngle', 'normal'); % 'italic'
    set(currentAxis, 'FontSize', 20);
    axis([min(rXtraj)-5 max(rXtraj)+5 min(rYtraj)-5 max(rYtraj)+5]);
    %axis(currentAxis, 'equal');
    %
    currentFigure=gcf;
    figName=sprintf('Solltrajektorie');
    set(currentFigure, 'Name', figName);
    scrsz = get(0,'ScreenSize');
    set(currentFigure, 'Position', [scrsz(4)*1/20 scrsz(4)*1/20 scrsz(4)*11/9 scrsz(4)*8/9]);
    %--------------------------------------------------------------------------
    % Save as PDF
    scrsz= get(0,'ScreenSize');
    set(gcf, 'PaperUnits', 'points');
    set(gcf, 'PaperSize', [scrsz(4)*9/9 scrsz(4)*2/9]);%[scrsz(4)*6/9 scrsz(4)*6/9]); %
    set(gcf, 'PaperPositionMode', 'manual');
    set(gcf, 'PaperPosition', [0 0 scrsz(4)*9/9 scrsz(4)*2/9]);%[0 0 scrsz(4)*6/9 scrsz(4)*6/9]); %
    set(gcf, 'renderer', 'painters');
    print(gcf, '-dpdf', 'FirstTrajectoryMultPoints3Stages.pdf');
%}



%--------------------------------------------------------------------------
% THIRD STAGE: COMPUTATION OF THE DRIVABLE REFERENCE TRAJECTORY BY USING A
% SIMPLE ONE-TRACK MODEL WITH CONTROLLERS TO FOLLOW THE TRAJECTORY FROM
% STAGE ONE AND TWO
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% EGO-VEHICLE PARAMETERS:
%--------------------------------------------------------------------------
lv=1.2; % distance from center of gravity to front axle of EGO in m
lh=1.5; % distance from center of gravity to rear axle of EGO in m
ell=lv+lh; % wheelbase EGO in m
w=1.8; % with of EGO
m=1500; % mass of EGO in kg
g=9.81; % gravitational acceleration in m/s^2
iL=16; % steering ration EGO
mu=1.0; % maximal adhesion coefficient

%--------------------------------------------------------------------------
% GEOMETRIC DATA FOR PLOTTING THE EGO AS RECTANGLE:
%--------------------------------------------------------------------------
lvToFront=0.85; % distance from front axle of EGO to front of car
lhToRear=1.1; % distance from rear axle of EGO to rear of car

%--------------------------------------------------------------------------
% INITIALIZATION OF VARIABLES FOR EGO:
%--------------------------------------------------------------------------
tPrediction=t1; % prediction time horizon in s
T=0.001;% discretization time in
t=0:T:tPrediction; % prediction time in s
N=size(t,2); % number of predition steps
rX=zeros(N,1); % X-position of EGO in global frame E in m
rY=zeros(N,1); % Y-position of EGO in global frame E in m
ax=zeros(N,1); % x-acceleration of EGO in E expressed in vehicle frame F in m/s^2
ay=zeros(N,1); % y-acceleration of EGO in E expressed in vehicle frame F in m/s^2
v=zeros(N,1); % magnitude of velocity of EGO over ground in m/s
dotV=0; % time derivative of magnitude of EGO velocity in m/s^2
beta=zeros(N,1); % slip angle of EGO in vehicle frame F in rad
dotBeta=0; % time derivative of slip angle in rad/s
gear=zeros(N,1); % gear
Psi=zeros(N,1); % yaw angle of EGO in global frame E in rad
dotPsi=zeros(N,1); % yaw rate of EGO in global frame E in rad/s
deltaL=zeros(N,1); % steering angle of EGO in rad
pB=zeros(N,1); % position of EGO brake pedal in percent
pG=zeros(N,1); % position of EGO throttle pedal in percent

%--------------------------------------------------------------------------
% INITIAL CONDITIONS OF EGO (which are not already defined in meneuver type):
%--------------------------------------------------------------------------
gear(1:end)=5; % has to be changed to fit the initial velocity vt0
v(1)=vt0;
Psi(1)=Psit0;
deltaL(1)=iL*atan2(ayt0*ell, vt0^2); % dotPsi=ay/v and dotPsi=(v*tan(deltaL/iL))/ell
dotPsi(1)=(v(1)*tan(deltaL(1)/iL))/ell;
ax(1)=dotV*cos(beta(1))-v(1)*sin(beta(1))*(dotBeta+dotPsi(1));
ay(1)=dotV*sin(beta(1))+v(1)*cos(beta(1))*(dotBeta+dotPsi(1));

%--------------------------------------------------------------------------
% COMPUTATION OF MOTION
%--------------------------------------------------------------------------
lastValidN=N; % only used for plotting the last position of the vehicle
standstill=false;
strongDrift=false;
for n=2:N
    %----------------------------------------------------------------------
    % COMPUTATION OF VALUES ACCORDING TO THE DESIRED TRAJECTORY USING A CONTROLLER
    %----------------------------------------------------------------------
    % time point in future for controller: tuning parameter
    if v(n-1)<(30/3.6)
        tVoraus=2-((2-0.3)/(30/3.6))*v(n-1);
    else
        tVoraus=0.3;
    end
    nTrajVoraus=min(ceil(n*T/Ttraj+tVoraus/Ttraj), Ntraj);
    if nTrajVoraus == Ntraj% end of trajectory reached during tVoraus
        tVoraus=tTraj(Ntraj)-t(n);
    end
    % simple prediction of position, velocity and orientation for the prediction time tVoraus
    rXtrajprime=rX(n-1)+v(n-1)*cos(Psi(n-1)+beta(n-1))*tVoraus + (ax(n-1)*cos(Psi(n-1))-ay(n-1)*sin(Psi(n-1)))*(tVoraus^2)/2;
    vXtrajprime=v(n-1)*cos(Psi(n-1)+beta(n-1))+(ax(n-1)*cos(Psi(n-1))-ay(n-1)*sin(Psi(n-1)))*tVoraus; % only forward motion
    rYtrajprime=rY(n-1)+v(n-1)*sin(Psi(n-1)+beta(n-1))*tVoraus + (ax(n-1)*sin(Psi(n-1))+ay(n-1)*cos(Psi(n-1)))*(tVoraus^2)/2;
    vYtrajprime=v(n-1)*sin(Psi(n-1)+beta(n-1))+(ax(n-1)*sin(Psi(n-1))+ay(n-1)*cos(Psi(n-1)))*tVoraus;
    PsiTrajprime=atan2(vYtrajprime, vXtrajprime);
    vTrajPrime=sqrt(vXtrajprime^2+vYtrajprime^2);
    
    % LONGITUDINAL ACCELERATRION CONTROLLER VELOCITY
    KGv=(T/0.001)*5;%0.5;
    KBv=(T/0.001)*5;%0.5;
    ev=vTraj(nTrajVoraus)-vTrajPrime;
    if ev>0
        pG(n)=min(pG(n-1)+KGv*ev, 100);
        pB(n)=0;
    elseif ev<0
        pG(n)=0;
        pB(n)=min(pB(n-1)+KBv*(-ev), 100);
    else
        pG(n)=0;
        pB(n)=0;
    end
    
    %    % LONGITUDINAL ACCELERATRION CONTROLLER POSITION:
    %     KGx=(T/0.001)*5;
    %     KBx=(T/0.001)*5;
    %     dVoraus=[rXtraj(nTrajVoraus); rYtraj(nTrajVoraus)]-[rXtrajprime; rYtrajprime];
    %     vTrajLongControl=[vXtraj(nTrajVoraus); vYtraj(nTrajVoraus)];
    %     ex=(dVoraus'*vTrajLongControl)/(vTrajLongControl'*vTrajLongControl);
    %     if ex>0
    %         pG(n)=min(pG(n-1)+KGx*ex, 100);
    %         pB(n)=0;
    %     elseif ex<0
    %         pG(n)=0;
    %         pB(n)=min(pB(n-1)+KBx*(-ex), 100);
    %     end
    
    % Low-pass filtering (PT1) of pG and pB
    pGNew=pG(n);
    pG(n)=pGNew+exp(-T/0.01)*(pG(n-1)-pGNew); % PT1 for pG
    pBNew=pB(n);
    pB(n)=pBNew+exp(-T/0.01)*(pB(n-1)-pBNew); % PT1 for pB
    
    % Allow only a maximum gradient for the brake pedal and the throttle change:
    maxGradiantPG=100/0.2; % maximal braking pedal gradient: 100% in 0.2 seconds;
    if abs(pG(n)-pG(n-1))/T > maxGradiantPG
        pG(n)=pG(n-1) + sign(pG(n)-pG(n-1))*(maxGradiantPG*T);
    end
    maxGradiantPB=100/0.2; % maximal throttle pedal gradient: 100% in 0.2 seconds;
    if abs(pB(n)-pB(n-1))/T > maxGradiantPB
        pB(n)=pB(n-1) + sign(pB(n)-pB(n-1))*(maxGradiantPB*T);
    end
    
    % Activate either braking or throttle pedal
    if pG(n)>pB(n)
        pB(n)=0;
    elseif pG(n)<pB(n)
        pG(n)=0;
    end
    
    
    % STEERING CONTROLLER:
    Kpsi=(T/0.001)*2;
    ePsi=PsiTraj(nTrajVoraus)-PsiTrajprime;
    
    Ky=(T/0.001)*(-15);
    dVoraus=[rXtraj(nTrajVoraus); rYtraj(nTrajVoraus)]-[rXtrajprime; rYtrajprime];
    vTrajOrtho=[vYtraj(nTrajVoraus); -vXtraj(nTrajVoraus)];
    ey=(dVoraus'*vTrajOrtho)/(vTrajOrtho'*vTrajOrtho);
    
    deltaL(n)=deltaL(n-1) + Kpsi*ePsi + Ky*ey;
    if deltaL(n)>720*pi/180
        deltaL(n)=720*pi/180;
    elseif deltaL(n)<-720*pi/180
        deltaL(n)=-720*pi/180;
    end
    
    % Low-pass filtering (PT1) for deltaL(n)
    deltaLNew=deltaL(n);
    deltaL(n)=deltaLNew+exp(-T/0.01)*(deltaL(n-1)-deltaLNew); % PT1 for deltaL
    
    maxGradiantDelta=(180/0.5)*pi/180; % maximal steering gradient: 180 degree in 0.5 seconds;
    if abs(deltaL(n)-deltaL(n-1))/T > maxGradiantDelta
        deltaL(n)=deltaL(n-1) + sign(Kpsi*ePsi + Ky*ey)*(maxGradiantDelta*T);
    end
    
    
    
    %----------------------------------------------------------------------
    % CONTINUE WITH SIMPLE VEHICLE MODEL
    %----------------------------------------------------------------------
    
    % Mapping from pG and pB to ax
    mu=1.0;
    if pG(n)>0
        amaxDryRoad=4;
        % amaxDryRoad= mappingVelocityToAmaxDry(v(n-1)); % maximal acceleration is dependent on the velocits => see Ex. 3.11 and 3.12
        aMax=mu*amaxDryRoad; 
        ax(n)=aMax*pG(n)/100;
    elseif pB(n)>0
        aMin=-1.0*mu*9.81;
        ax(n)=aMin*pB(n)/100;
    end
    
    % Difference equations:
    beta(n)=0;
    v(n)=v(n-1)+T*ax(n);
    if ((v(n)-v(n-1)<0) && (abs(v(n))<1)) || standstill
        v(n:end)=0;
        ax(n)=0;
        dotPsi(n)=0;
        Psi(n)=Psi(n-1);
        ay(n)=0;
        rX(n)=rX(n-1);
        rY(n)=rY(n-1);
        standstill=true;
    else
        dotPsi(n)=(v(n)*tan(deltaL(n)/iL))/ell;
        Psi(n)=Psi(n-1)+dotPsi(n)*T;
        ay(n)=dotPsi(n)*v(n);
        rX(n)=rX(n-1) + v(n)*cos(Psi(n))*T + ax(n)*cos(Psi(n))*0.5*T^2 - ay(n)*sin(Psi(n))*0.5*T^2;
        rY(n)=rY(n-1) + v(n)*sin(Psi(n))*T + ax(n)*sin(Psi(n))*0.5*T^2 + ay(n)*cos(Psi(n))*0.5*T^2;
    end
end

toc


%--------------------------------------------------------------------------
% VISUALIZATION OF FINAL REFERENCE TRAJECTORY
%--------------------------------------------------------------------------

%--------------------------------
% Visualization of inputs to EGO
%--------------------------------
figure;

subplot(4,1,1);
h1=plot(t, pG, 'g', 'LineWidth', 2);
%leg1=legend(h1,{'$bla$'}); set(leg2,'Interpreter','latex');
%legend('Location','northeast');
% xticks(0:tPrediction); %xticklabels({'t = 0','t = 2','t = 3'})
% if min(pG)~=max(pG)
%     yticks([min(pG) 0.5*(min(pG)+max(pG)) max(pG)]); %yticklabels({'bla','bla'})
%     axis([0 tPrediction min(pG) max(pG)]);
% else
%     yticks([max(pG)-1 0.5*((max(pG)-1)+(max(pG)+1)) max(pG)+1]);
%     axis([0 tPrediction max(pG)-1 max(pG)+1]);
% end
xlabel('$t$ in s', 'Interpreter','latex');
ylabel('$p_\mathrm{G}$ in $\%$','Interpreter','latex');
currentAxis=gca;
set(currentAxis, 'FontName', 'Times New Roman');
set(currentAxis, 'FontAngle', 'normal'); % 'italic'
set(currentAxis, 'FontSize', 15);

subplot(4,1,2);
h2=plot(t, pB, 'r', 'LineWidth', 2);
%leg2=legend(h2,{'$bla$'}); set(leg2,'Interpreter','latex');
%legend('Location','northeast');
% xticks(0:tPrediction); %xticklabels({'t = 0','t = 2','t = 3'})
% if min(pB)~=max(pB)
%     yticks([min(pB) 0.5*(min(pB)+max(pB)) max(pB)]); %yticklabels({'bla','bla'})
%     axis([0 tPrediction min(pB) max(pB)]);
% else
%     yticks([max(pB)-1 0.5*((max(pB)-1)+(max(pB)+1)) max(pB)+1]);
%     axis([0 tPrediction max(pB)-1 max(pB)+1]);
% end
xlabel('$t$ in s', 'Interpreter','latex');
ylabel('$p_\mathrm{B}$ in $\%$','Interpreter','latex');
currentAxis=gca;
set(currentAxis, 'FontName', 'Times New Roman');
set(currentAxis, 'FontAngle', 'normal'); % 'italic'
set(currentAxis, 'FontSize', 15);

subplot(4,1,3);
deltaLGrad=deltaL*180/pi;
h3=plot(t, deltaLGrad, 'b', 'LineWidth', 2);
%leg3=legend(h3,{'$bla$'}); set(leg2,'Interpreter','latex');
%legend('Location','northeast');
% xticks(0:tPrediction); %xticklabels({'t = 0','t = 2','t = 3'})
% if min(deltaLGrad)~=max(deltaLGrad)
%     yticks([min(deltaLGrad) 0.5*(min(deltaLGrad)+max(deltaLGrad)) max(deltaLGrad)]); %yticklabels({'bla','bla'})
%     axis([0 tPrediction min(deltaLGrad) max(deltaLGrad)]);
% else
%     yticks([max(deltaLGrad)-1 0.5*((max(deltaLGrad)-1)+(max(deltaLGrad)+1)) max(deltaLGrad)+1]);
%     axis([0 tPrediction max(deltaLGrad)-1 max(deltaLGrad)+1]);
% end
xlabel('$t$ in s', 'Interpreter','latex');
ylabel('$\delta_{\mathrm{L}}$ in $^\circ$','Interpreter','latex');
currentAxis=gca;
set(currentAxis, 'FontName', 'Times New Roman');
set(currentAxis, 'FontAngle', 'normal'); % 'italic'
set(currentAxis, 'FontSize', 15);

subplot(4,1,4);
h4=plot(t, gear, 'k', 'LineWidth', 2);
%leg4=legend(h4,{'$bla$'}); set(leg2,'Interpreter','latex');
%legend('Location','northeast');
% xticks(0:tPrediction); %xticklabels({'t = 0','t = 2','t = 3'})
% if min(gear)~=max(gear)
%     yticks([min(gear) 0.5*(min(gear)+max(gear)) max(gear)]); %yticklabels({'bla','bla'})
%     axis([0 tPrediction min(gear) max(gear)]);
% else
%     yticks([max(gear)-1 0.5*((max(gear)-1)+(max(gear)+1)) max(gear)+1]);
%     axis([0 tPrediction max(gear)-1 max(gear)+1]);
% end
xlabel('$t$ in s', 'Interpreter','latex');
ylabel('Gang','Interpreter','latex');
currentAxis=gca;
set(currentAxis, 'FontName', 'Times New Roman');
set(currentAxis, 'FontAngle', 'normal'); % 'italic'
set(currentAxis, 'FontSize', 15);
%
currentFigure=gcf;
figName=sprintf('Inputs to EGO and gear');
set(currentFigure, 'Name', figName);
scrsz = get(0,'ScreenSize');
set(currentFigure, 'Position', [scrsz(4)*1/20 scrsz(4)*1/20 scrsz(4)*11/9 scrsz(4)*7.5/9]);
%--------------------------------------------------------------------------
% Save as PDF
scrsz= get(0,'ScreenSize');
set(gcf, 'PaperUnits', 'points');
set(gcf, 'PaperSize', [scrsz(4)*9/9 scrsz(4)*6/9]);%[scrsz(4)*6/9 scrsz(4)*6/9]); %
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 scrsz(4)*9/9 scrsz(4)*6/9]);%[0 0 scrsz(4)*6/9 scrsz(4)*6/9]); %
set(gcf, 'renderer', 'painters');
%print(gcf, '-dpdf', 'TrajectoryControlInputsEGOMultPoints3Stages.pdf');


%-----------------------------------
% Visualization of quantities of EGO
%-----------------------------------
figure;

subplot(7,1,1);
h1=plot(t, ax, 'r', 'LineWidth', 2);
%leg1=legend(h1,{'$bla$'}); set(leg1,'Interpreter','latex');
%legend('Location','northeast');
% xticks(0:tPrediction); %xticklabels({'t = 0','t = 2','t = 3'})
% if min(ax)~=max(ax)
%     yticks([min(ax) 0.5*(min(ax)+max(ax)) max(ax)]); %yticklabels({'bla','bla'})
%     axis([0 tPrediction min(ax) max(ax)]);
% else
%     yticks([max(ax)-1 0.5*((max(ax)-1)+(max(ax)+1)) max(ax)+1]);
%     axis([0 tPrediction max(ax)-1 max(ax)+1]);
% end
xlabel('$t$ in s', 'Interpreter','latex');
ylabel('$a_x$ in $\frac{\mathrm{m}}{\mathrm{s}^2}$','Interpreter','latex');
currentAxis=gca;
set(currentAxis, 'FontName', 'Times New Roman');
set(currentAxis, 'FontAngle', 'normal'); % 'italic'
set(currentAxis, 'FontSize', 15);

subplot(7,1,2);
h2=plot(t, ay, 'b', 'LineWidth', 2);
%leg2=legend(h2,{'$bla$'}); set(leg2,'Interpreter','latex');
%legend('Location','northeast');
% xticks(0:tPrediction); %xticklabels({'t = 0','t = 2','t = 3'})
% if min(ay)~=max(ay)
%     yticks([min(ay) 0.5*(min(ay)+max(ay)) max(ay)]); %yticklabels({'bla','bla'})
%     axis([0 tPrediction min(ay) max(ay)]);
% else
%     yticks([max(ay)-1 0.5*((max(ay)-1)+(max(ay)+1)) max(ay)+1]);
%     axis([0 tPrediction max(ay)-1 max(ay)+1]);
% end
xlabel('$t$ in s', 'Interpreter','latex');
ylabel('$a_y$ in $\frac{\mathrm{m}}{\mathrm{s}^2}$','Interpreter','latex');
currentAxis=gca;
set(currentAxis, 'FontName', 'Times New Roman');
set(currentAxis, 'FontAngle', 'normal'); % 'italic'
set(currentAxis, 'FontSize', 15);

subplot(7,1,3);
dotPsiGradPs=dotPsi*180/pi;
h3=plot(t, dotPsiGradPs, 'b', 'LineWidth', 2);
%leg3=legend(h3,{'$bla$'}); set(leg3,'Interpreter','latex');
%legend('Location','northeast');
% xticks(0:tPrediction); %xticklabels({'t = 0','t = 2','t = 3'})
% if min(dotPsiGradPs)~=max(dotPsiGradPs)
%     yticks([min(dotPsiGradPs) 0.5*(min(dotPsiGradPs)+max(dotPsiGradPs)) max(dotPsiGradPs)]); %yticklabels({'bla','bla'})
%     axis([0 tPrediction min(dotPsiGradPs) max(dotPsiGradPs)]);
% else
%     yticks([max(dotPsiGradPs)-1 0.5*((max(dotPsiGradPs)-1)+(max(dotPsiGradPs)+1)) max(dotPsiGradPs)+1]);
%     axis([0 tPrediction max(dotPsiGradPs)-1 max(dotPsiGradPs)+1]);
% end
xlabel('$t$ in s', 'Interpreter','latex');
ylabel('$\dot{\Psi}$ in $\frac{^\circ}{\mathrm{s}}$','Interpreter','latex');
currentAxis=gca;
set(currentAxis, 'FontName', 'Times New Roman');
set(currentAxis, 'FontAngle', 'normal'); % 'italic'
set(currentAxis, 'FontSize', 15);

subplot(7,1,4);
PsiGrad=Psi*180/pi;
h4=plot(t, PsiGrad, 'b', 'LineWidth', 2);
%leg4=legend(h4,{'$bla$'}); set(leg4,'Interpreter','latex');
%legend('Location','northeast');
% xticks(0:tPrediction); %xticklabels({'t = 0','t = 2','t = 3'})
% if min(PsiGrad)~=max(PsiGrad)
%     yticks([min(PsiGrad) 0.5*(min(PsiGrad)+max(PsiGrad)) max(PsiGrad)]); %yticklabels({'bla','bla'})
%     axis([0 tPrediction min(PsiGrad) max(PsiGrad)]);
% else
%     yticks([max(PsiGrad)-1 0.5*((max(PsiGrad)-1)+(max(PsiGrad)+1)) max(PsiGrad)+1]);
%     axis([0 tPrediction max(PsiGrad)-1 max(PsiGrad)+1]);
% end
xlabel('$t$ in s', 'Interpreter','latex');
ylabel('$\Psi$ in $^\circ$','Interpreter','latex');
currentAxis=gca;
set(currentAxis, 'FontName', 'Times New Roman');
set(currentAxis, 'FontAngle', 'normal'); % 'italic'
set(currentAxis, 'FontSize', 15);
%
subplot(7,1,5);
vKMH=v*3.6;
h5=plot(t,vKMH, 'g', 'LineWidth', 2);
%leg5=legend(h6,{'$bla$'}); set(leg5,'Interpreter','latex');
%legend('Location','northeast');
% xticks(0:tPrediction); %xticklabels({'t = 0','t = 2','t = 3'})
% if min(vKMH)~=max(vKMH)
%     yticks([min(vKMH) 0.5*(min(vKMH)+max(vKMH)) max(vKMH)]); %yticklabels({'bla','bla'})
%     axis([0 tPrediction min(vKMH) max(vKMH)]);
% else
%     yticks([max(vKMH)-1 0.5*((max(vKMH)-1)+(max(vKMH)+1)) max(vKMH)+1]);
%     axis([0 tPrediction max(vKMH)-1 max(vKMH)+1]);
% end
xlabel('$t$ in s', 'Interpreter','latex');
ylabel('$v$ in $\frac{\mathrm{km}}{\mathrm{h}}$','Interpreter','latex');
currentAxis=gca;
set(currentAxis, 'FontName', 'Times New Roman');
set(currentAxis, 'FontAngle', 'normal'); % 'italic'
set(currentAxis, 'FontSize', 15);
%
subplot(7,1,6);
h6=plot(t, rX, 'k', 'LineWidth', 2);
%leg6=legend(h6,{'$bla$'}); set(leg6,'Interpreter','latex');
%legend('Location','northeast');
% xticks(0:tPrediction); %xticklabels({'t = 0','t = 2','t = 3'})
% if min(rX)~=max(rX)
%     yticks([min(rX) 0.5*(min(rX)+max(rX)) max(rX)]); %yticklabels({'bla','bla'})
%     axis([0 tPrediction min(rX) max(rX)]);
% else
%     yticks([max(rX)-1 0.5*((max(rX)-1)+(max(rX)+1)) max(rX)+1]);
%     axis([0 tPrediction max(rX)-1 max(rX)+1]);
% end
xlabel('$t$ in s', 'Interpreter','latex');
ylabel('$r_X$ in m','Interpreter','latex');
currentAxis=gca;
set(currentAxis, 'FontName', 'Times New Roman');
set(currentAxis, 'FontAngle', 'normal'); % 'italic'
set(currentAxis, 'FontSize', 15);
%
subplot(7,1,7);
h7=plot(t, rY, 'k', 'LineWidth', 2);
%leg7=legend(h7,{'$bla$'}); set(leg7,'Interpreter','latex');
%legend('Location','northeast');
% xticks(0:tPrediction); %xticklabels({'t = 0','t = 2','t = 3'})
% if min(rY)~=max(rY)
%     yticks([min(rY) 0.5*(min(rY)+max(rY)) max(rY)]); %yticklabels({'bla','bla'})
%     axis([0 tPrediction min(rY) max(rY)]);
% else
%     yticks([max(rY)-1 0.5*((max(rY)-1)+(max(rY)+1)) max(rY)+1]);
%     axis([0 tPrediction max(rY)-1 max(rY)+1]);
% end
xlabel('$t$ in s', 'Interpreter','latex');
ylabel('$r_Y$ in m','Interpreter','latex');
currentAxis=gca;
set(currentAxis, 'FontName', 'Times New Roman');
set(currentAxis, 'FontAngle', 'normal'); % 'italic'
set(currentAxis, 'FontSize', 15);
%
currentFigure=gcf;
figName=sprintf('EGO states over time');
set(currentFigure, 'Name', figName);
scrsz = get(0,'ScreenSize');
set(currentFigure, 'Position', [scrsz(4)*1/20 scrsz(4)*1/20 scrsz(4)*11/9 scrsz(4)*7.5/9]);
%--------------------------------------------------------------------------
% Save as PDF
scrsz= get(0,'ScreenSize');
set(gcf, 'PaperUnits', 'points');
set(gcf, 'PaperSize', [scrsz(4)*9/9 scrsz(4)*6/9]);%[scrsz(4)*6/9 scrsz(4)*6/9]); %
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 scrsz(4)*9/9 scrsz(4)*6/9]);%[0 0 scrsz(4)*6/9 scrsz(4)*6/9]); %
set(gcf, 'renderer', 'painters');
% print(gcf, '-dpdf', 'TrajectoryControlOverTimeEGOMultPoints3Stages.pdf');


%-----------------------------------
% Visualization of trajectory of EGO
%-----------------------------------
figure;
h1=plot(rX, rY, 'k', 'LineWidth', 2); % Plot trajectory of EGO
%leg1=legend(h1,{'$r_Y$ vs. $r_X$'}); set(leg1,'Interpreter','latex');
%legend('Location','northeast');
%xticks([0 10 20 30 40]); %xticklabels({'rX=0','rX=10','rX=20','rX=30','rX=40'})
%yticks([0 2 4 6 8]); %yticklabels({'r_Y=0','r_Y=2','r_Y=4','r_Y=6','r_Y=8'})
%axis([0 40 0 8]);
xlabel('$r_X$ in m', 'Interpreter','latex');
ylabel('$r_Y$ in m','Interpreter','latex');
hold on;
%
countPlotBodyShell=0;
oldDistance=0;
carStopped1=0;
plotDistance=6;
xthBodyShellWithText=3;
for n=1:N
    % CORNERS OF EGO-CAR:
    ex=cos(Psi(n));
    ey=sin(Psi(n));
    exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
    eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
    P_vl_X=rX(n)+ex*(lv+lvToFront)+w/2*exOrtho; % X-coordinate in E of the front left point of the car
    P_vl_Y=rY(n)+ey*(lv+lvToFront)+w/2*eyOrtho; % Y-coordinate in E of the front left point of the car
    P_vr_X=rX(n)+ex*(lv+lvToFront)-w/2*exOrtho; % X-coordinate in E of the front right point of the car
    P_vr_Y=rY(n)+ey*(lv+lvToFront)-w/2*eyOrtho; % Y-coordinate in E of the front right point of the car
    P_rl_X=rX(n)-ex*(lh+lhToRear)+w/2*exOrtho; % X-coordinate in E of the rear left point of the car
    P_rl_Y=rY(n)-ey*(lh+lhToRear)+w/2*eyOrtho; % Y-coordinate in E of the rear left point of the car
    P_rr_X=rX(n)-ex*(lh+lhToRear)-w/2*exOrtho; % X-coordinate in E of the rear right point of the car
    P_rr_Y=rY(n)-ey*(lh+lhToRear)-w/2*eyOrtho; % Y-coordinate in E of the rear right point of the car
    %
    % VELOCITY VECTOR OF EGO-CAR
    exVel=cos(Psi(n)+beta(n));
    eyVel=sin(Psi(n)+beta(n));
    %
    % VISUALIZE ONLY SOME OF THE COMPUTED VALUES:
    if (n>1) && (v(n)==0 && v(n-1)~=0)
        carStopped1=1;
    end
    if n>1
        oldDistance=oldDistance+sqrt((rX(n)-rX(n-1))^2+(rY(n)-rY(n-1))^2);
    end
    if (n==1) || (oldDistance > plotDistance) || (n==lastValidN) %|| (carStopped1==1) % tunig parameter
        oldDistance=0;
        %PLOT EGO
        plot([P_vl_X P_vr_X], [P_vl_Y P_vr_Y], 'b', 'LineWidth', 4);hold on; % plot front line of car
        plot([P_vl_X P_rl_X], [P_vl_Y P_rl_Y], 'b', 'LineWidth', 2);hold on; % plot left line of car
        plot([P_vr_X P_rr_X], [P_vr_Y P_rr_Y], 'b', 'LineWidth', 2);hold on; % plot right line of car
        plot([P_rr_X P_rl_X], [P_rr_Y P_rl_Y], 'b', 'LineWidth', 2);hold on; % plot rear line of car
        plot([rX(n) rX(n)+exVel*v(n)],[rY(n) rY(n)+eyVel*v(n)], 'g'); hold on; % plot velocity vector in green
        plot(rX(n),rY(n), 's',  'MarkerEdgeColor','k', 'MarkerFaceColor','b', 'MarkerSize',6); % plot center of gravity with square marker
        % SET TEXT ONLY TO SOME OF THE CAR VISUALIZATIONS:
        countPlotBodyShell=countPlotBodyShell+1;
        if (rem(countPlotBodyShell,xthBodyShellWithText)==0) || (n==1) || (n==lastValidN) %|| (carStopped1==1)
            % Text on EGO
            if ~isnan(Psi(n)) && ~isnan(v(n))
                text(P_rl_X-5*exOrtho, P_rl_Y-5*eyOrtho, sprintf('$t=%2.1f$s',t(n)), 'Rotation', Psi(n)*180/pi, 'FontSize', 17, 'Interpreter','latex');
                text(P_rl_X-3*exOrtho, P_rl_Y-3*eyOrtho, sprintf('$v=%2.0f \\frac{\\mathrm{km}}{\\mathrm{h}}$',3.6*v(n)), 'Rotation', Psi(n)*180/pi, 'FontSize', 17, 'Interpreter','latex');
            end
        end
    end
end

hold on; plot(rXtraj, rYtraj, 'm', 'LineWidth',1, 'LineStyle', '--');

currentAxis=gca;
set(currentAxis, 'FontName', 'Times New Roman');
set(currentAxis, 'FontAngle', 'normal'); % 'italic'
set(currentAxis, 'FontSize', 20);
axis([min(rXtraj)-5 max(rXtraj)+5 min(rYtraj)-5 max(rYtraj)+5]);
%axis(currentAxis, 'equal');
%
currentFigure=gcf;
figName=sprintf('Umgesetzte Trajektorie Fahrzeug');
set(currentFigure, 'Name', figName);
scrsz = get(0,'ScreenSize');
%set(currentFigure, 'Position', [scrsz(4)*1/20 scrsz(4)*1/20 scrsz(4)*11/9 scrsz(4)*8/9]);
set(currentFigure, 'Position', [scrsz(4)*1/20 scrsz(4)*1/20 scrsz(4)*15/9 scrsz(4)*3/9]);
%--------------------------------------------------------------------------
% Save as PDF
scrsz= get(0,'ScreenSize');
set(gcf, 'PaperUnits', 'points');
set(gcf, 'PaperSize', [scrsz(4)*9/9 scrsz(4)*2/9]);%[scrsz(4)*6/9 scrsz(4)*6/9]); %
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 scrsz(4)*9/9 scrsz(4)*2/9]);%[0 0 scrsz(4)*6/9 scrsz(4)*6/9]); %
set(gcf, 'renderer', 'painters');
% print(gcf, '-dpdf', 'TrajectoryControlTrajectoryMultPoints3Stages.pdf');




%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

% %--------------------------------------------------------------------------
% % HELP FUNCTION FOR MAXIMAL ACCELERATION ON DRY ROAD DEPENDING ON VELOCITY
% %--------------------------------------------------------------------------
% function aMAxDry=mappingVelocityToAmaxDry(v)
% vValues=(0:10:200)/3.6;
% aMaxDryValues=[3 4 5 6 7 6.5 5.5 3.5 3 2.5 2 1.9 1.8 1.7 1.5 1.4 1.1 0.9 0.7 0.5 0.3]; % see e.g. Exercise 3.11
% 
% aMAxDry=interp1(vValues, aMaxDryValues, v);
% end
% %--------------------------------------------------------------------------