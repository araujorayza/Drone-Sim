%% SCRIPT FOR CLOSED LOOP SIMULATION OF THE DRONE SYSTEM
clear all
clc
close all

%% SYSTEM INIT AND SIM PARAMETERS

% ChangeGamma
InitSTATE=[0;0;0;0;.5;0;1;0];
sphinx = true; %flag for which gamma we use

simStep = 0.25;
Tfinal  = 25;

TRAJECTORY      = 'circle';
% TRAJECTORY = 'LemniscataBernoulli';


% ControlType     = 'FuzzywithParamUncertainty';
ControlType     = 'FuzzyParamUncertaintyDisturbance';
% ControlType='StateFeedback';
% ControlType='LQR';
% ControlType='FeedbackLin';
% ControlType='Fuzzy';
% ControlType='FuzzyWithZuncertainty';


MaxRotSpd       = 100; %max rotation speed in deg/s (yaw rate)
MaxXSpd         = 0.5; %max X speed in m/s 
MaxYSpd         = 0.5; %max Y speed in m/s 
MaxZSpd         = 0.5; %max Z speed in m/s 

% Saturation on the control signal
Saturation = false;

% Wind disturbance simulation
Wind_Disturbance = false;
W_static = [0.03;
            0.03;
            0.03;
            0.03];
a=10*[0.03 0.03 0.03];
Gammas=[0 0 0];
Omegas=[2.2 2.5 2.7];
DisturbanceTime = [3 Tfinal];

%% Load Parameters and set time vector
t=0:simStep:Tfinal;
if(sphinx)
    load gamma_sphinx
else
    load gamma_bebop
end


SimStruct.trajectory=TRAJECTORY;
SimStruct.sys.param = gamma;
SimStruct.WindDisturbance.Type = Wind_Disturbance;
SimStruct.WindDisturbance.Ws = W_static;
SimStruct.WindDisturbance.a = a;
SimStruct.WindDisturbance.Gammas=Gammas;
SimStruct.WindDisturbance.Omegas=Omegas;
SimStruct.WindDisturbance.time=DisturbanceTime;

%% CONTROL DESIGN
ControlDesign
SimStruct.controller=ControllerStruct;

%% TRAJECTORY PLANNING
[q_d,dq_d,ddq_d]=CalcDesTrajectory(TRAJECTORY,t);
DES_STATE = [dq_d;q_d]';

%% SYSTEM SIMULATION
STATE = ode4(@DRONE_SANTANA,t,InitSTATE,...                            %TRAJECTORY, ControllerStruct, gamma);
                                 SimStruct);

% [t,STATE] = ode45(@DRONE_SANTANA,t,InitSTATE,[],...                            %TRAJECTORY, ControllerStruct, gamma);
%                                  SimStruct);
InitERROR = InitSTATE-[dq_d(:,1);q_d(:,1)];
[ERROR] = ode4(@ERROR_Fuzzy,t,InitERROR,...
                                SimStruct);                                    %TRAJECTORY, ControllerStruct, gamma);
%% OUTPUT PLOTTING
gamma
InitSTATE
simStep
Tfinal
TRAJECTORY
Controller      = ControllerStruct.type
ControllerSat   = ControllerStruct.sat

%
VelX    = 1;
VelY    = 2;
VelZ    = 3;
VelYaw  = 4;
PosX    = 5;
PosY    = 6;
PosZ    = 7;
PosYaw  = 8;

% QualiSimPlot

%%%%%%%%%%%%%%%%%TRAJECTORY PLOT%%%%%%%%%%%%%%%%%%%%%%%
figure(2);

hold on;
plot(STATE(:,PosX),STATE(:,PosY),'*');
plot(ERROR(:,PosX)+ DES_STATE(:,PosX),ERROR(:,PosY) + DES_STATE(:,PosY),'rd');
plot(DES_STATE(:,PosX),DES_STATE(:,PosY),'k*');
hold off
grid on
axis equal
legend('Trajectory')
legend('NonLinSys','FuzzyErrorSys','Ref')

%%%%%%%%%%%%%%%%%CONTROL SIGNAL PLOT%%%%%%%%%%%%%%%%%%%%%%%
figure(3);
U = zeros(4,length(t));
V = U;

for i = 1:length(t)
    U(:,i) = CalcVirtControlLaw(ControllerStruct,t(i),STATE(i,:)',ddq_d(:,i),dq_d(:,i),q_d(:,i));
%     V(:,i)=M\(U(:,i) + N*R'*dq_d + ddq_d);
%     
%     if (controller.sat)
%         V=saturate_control(V);
%     end
end 

plot(t',U,'d');
grid on
title('Control Signal')
legend('U_x','U_y','U_z','U_y_a_w');


%%%%%%%%%%%%%%%%%COMPONENT PLOT%%%%%%%%%%%%%%%%%%%%%%%
Knew=gen_K4cpp(K);
[K{1};K{2};K{3};K{4}]

figure(4);
hold on
plot(t,STATE(:,PosX),'*');
plot(t,ERROR(:,PosX)+ DES_STATE(:,PosX),'rd');
plot(t,DES_STATE(:,PosX),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('x(t)')

figure(5);
hold on
plot(t,STATE(:,PosY),'*');
plot(t,ERROR(:,PosY)+ DES_STATE(:,PosY),'rd');
plot(t,DES_STATE(:,PosY),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('y(t)')


figure(6);
hold on
plot(t,STATE(:,PosZ),'*');
plot(t,ERROR(:,PosZ)+ DES_STATE(:,PosZ),'rd');
plot(t,DES_STATE(:,PosZ),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('z(t)')

figure(7);
hold on
plot(t,STATE(:,PosYaw),'*');
plot(t,ERROR(:,PosYaw)+ DES_STATE(:,PosYaw),'rd');
plot(t,DES_STATE(:,PosYaw),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('yaw(t)')

if(SimStruct.WindDisturbance.Type)
    h=8;
    figure(h);
    D = zeros(4,length(t));

    for i = 1:length(t)
        D(:,i) = WindDisturbance(t(i),SimStruct.WindDisturbance);
    end 

    plot(t',D(1,:));
    grid on
    % legend('$D_x$','$D_y$','$D_z$','$D_{\psi}$');
    legend('Disturbance Signal')
end 


figure(10);
hold on
plot(t,STATE(:,VelX),'*');
plot(t,ERROR(:,VelX)+ DES_STATE(:,VelX),'rd');
plot(t,DES_STATE(:,VelX),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('yaw(t)')

%%
% GeneratePlots;



