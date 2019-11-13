%% SCRIPT FOR CLOSED LOOP SIMULATION OF THE DRONE SYSTEM
clear all
clc
close all

%% SYSTEM INIT AND SIM PARAMETERS

% ChangeGamma
InitSTATE=0*[0;0;0;0;1.5;1;1;0];

simStep = 0.1;
Tfinal = 30;


% TRAJECTORY   = 'circle';
% TRAJECTORY = 'LemniscataBernoulli';

% ControlType='StateFeedback';
% ControlType='LQR';
% ControlType='FeedbackLin';
% ControlType='Fuzzy';
% ControlType='FuzzyWithZuncertainty';

TRAJECTORY      = 'circle';
ControlType     = 'Fuzzy';

% Saturation on the control signal
Saturation = false;
%% Load Parameters and set time vector
t=0:simStep:Tfinal;
load gamma
%% CONTROL DESIGN
ControlDesign
%% SYSTEM SIMULATION
[q_d,dq_d,ddq_d]=CalcDesTrajectory(TRAJECTORY,t);

STATE = ode4(@DRONE_SANTANA,t,InitSTATE,...
                                TRAJECTORY, ControllerStruct, gamma);

InitERROR = InitSTATE-[dq_d(:,1);q_d(:,1)];
ERROR = ode4(@ERROR_Fuzzy,t,InitERROR,...
                                TRAJECTORY, ControllerStruct, gamma);
%% OUTPUT PLOTTING
gamma
InitSTATE
simStep
Tfinal
TRAJECTORY
ControllerStruct.type
ControllerStruct.sat

DES_STATE = [dq_d;q_d]';

% QualiSimPlot

plot(DES_STATE(:,5),DES_STATE(:,6),'*');
figure;
hold on;
plot(STATE(:,5),STATE(:,6),'*');
plot(ERROR(:,5)+ DES_STATE(:,5),ERROR(:,6) + DES_STATE(:,6),'d');
hold off
grid on
axis equal
legend

Knew=gen_K4cpp(K);
[K{1};K{2};K{3};K{4}]
% gamma=gamma'
% figure;
% plot(t,q_d(4,:),'*');
% hold on;
% plot(t,STATE(:,8));
% grid on
% axis equal
% legend

% figure;
% hold on;
% DES_STATE = [dq_d;q_d]';
% 
% plot(t,DES_STATE(:,5),'*');
% plot(t,STATE(:,5));
% plot(t,ERROR(:,5)+ DES_STATE(:,5),'d');
% hold off
% grid on
% axis equal
% legend

% figure;
% hold on;
% DES_STATE = [dq_d;q_d]';
% 
% plot(t,DES_STATE(:,6),'*');
% plot(t,STATE(:,6));
% plot(t,ERROR(:,6)+ DES_STATE(:,6),'d');
% hold off
% grid on
% axis equal
% legend
