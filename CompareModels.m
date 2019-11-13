%% SCRIPT FOR COMPARISON OF ERROR MODELS
clear all
clc
close all

%% SYSTEM INIT AND SIM PARAMETERS

% ChangeGamma
InitSTATE=0*[0;0;0;0;1.5;1;1;0];

simStep = 0.1;
Tfinal = 10;


TRAJECTORY   = 'circle';
ControlType='Fuzzy';
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
ERROR_F = ode4(@ERROR_Fuzzy,t,InitERROR,...
                                TRAJECTORY, ControllerStruct, gamma);
ERROR_S = ode4(@ERROR_SANTANA,t,InitERROR,...
                                TRAJECTORY, ControllerStruct, gamma);
ERROR_T= STATE;
dERROR_F = zeros(length(t),8);
for i=1:length(t)
   dERROR_F(i,:) = ERROR_Fuzzy(t(i),ERROR_T(i,:)',TRAJECTORY, ControllerStruct, gamma);
end

dERROR_S = zeros(length(t),8);
for i=1:length(t)
   dERROR_S(i,:) = ERROR_SANTANA(t(i),ERROR_T(i,:)',TRAJECTORY, ControllerStruct, gamma);
end
%% OUTPUT PLOTTING
gamma
InitSTATE
simStep
Tfinal
TRAJECTORY
ControllerStruct.type
ControllerStruct.sat

figure;
hold on;
DES_STATE = [dq_d;q_d]';
DES_dSTATE = [ddq_d;dq_d]';

plot(t,dERROR_S(:,6),'*');
plot(t,dERROR_F(:,6),'d');
hold off
grid on
axis equal
legend

dERROR_S(:,6)-dERROR_F(:,6)
