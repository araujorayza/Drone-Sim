%SCRIPT FOR CLOSED LOOP SIMULATION OF THE DRONE SYSTEM
clear all
clc
close all
%% SYSTEM INIT AND SIM PARAMETERS

InitSTATE=[ 4;
            5;
            10;
            1.5;
            4;
            5;
            10;
            1.5];


simStep = 0.1;
Tfinal = 20;

% TRAJECTORY = 'LemniscataBernoulli';
TRAJECTORY   = 'circle';

ControlType='StateFeedback';
% ControlType='Fuzzy';
% ControlType='OpenLoop';
%% Load Parameters and set time vector
t=0:simStep:Tfinal;
load gamma

%% CONTROL DESIGN
ControlDesign 

%% SYSTEM SIMULATION
[q_d,dq_d,ddq_d]=CalcDesTrajectory(TRAJECTORY,t);

[t,STATE]=ode45(@(t,state) DRONE_SANTANA(t,state,...
                                        TRAJECTORY, ControllerStruct, gamma),...
                                         t,InitSTATE);
%% OUTPUT PLOTTING
plot(q_d(1,:),q_d(2,:),'*');
hold on;
plot(STATE(:,5),STATE(:,6));
hold off
grid on
axis equal
legend


% figure;
% plot(t,q_d(4,:),'*');
% hold on;
% plot(t,STATE(:,8));
% grid on
% axis equal
% legend


% figure;
% subplot(2,2,1);
% plot(t,q_d(1,:)'-STATE(:,5));
% title('error behavior for pos x')
% subplot(2,2,2);
% plot(t,q_d(2,:)'-STATE(:,6));
% title('error behavior for pos y')
% subplot(2,2,3);
% plot(t,q_d(3,:)'-STATE(:,7));
% title('error behavior for pos z')
% subplot(2,2,4);
% plot(t,q_d(4,:)'-STATE(:,8));
% title('error behavior for pos yaw')

