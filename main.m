%% SCRIPT FOR CLOSED LOOP SIMULATION OF THE DRONE SYSTEM
clear all
clc
close all

VelX    = 1;
VelY    = 2;
VelZ    = 3;
VelYaw  = 4;
PosX    = 5;
PosY    = 6;
PosZ    = 7;
PosYaw  = 8;



%% SYSTEM INIT AND SIM PARAMETERS

% ChangeGamma
InitSTATE=rand([8,1]); %[0;0;0;0;0;0;0;0];
sphinx = true; %flag for which gamma we use
tol = 12;

simStep = 0.25;
Tfinal  = 25;

% TRAJECTORY      = 'circle';
TRAJECTORY      = 'openloop';
% TRAJECTORY = 'LemniscataBernoulli';


% ControlType     = 'FuzzywithParamUncertainty';
% ControlType     = 'FuzzyParamUncertaintyDisturbance';
% ControlType='StateFeedback';
% ControlType='LQR';
% ControlType='FeedbackLin';
% ControlType = 'Fuzzy';
ControlType = 'openloop';
% ControlType='FuzzyWithZuncertainty';

Modeltype = 1;

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

%% TRAJECTORY PLANNING
[q_d,dq_d,ddq_d]=CalcDesTrajectory(TRAJECTORY,t);
DES_STATE = [dq_d;q_d]';

%% SYSTEM SIMULATION
InitERROR = InitSTATE - DES_STATE(1,:)';


% STATE = ode4(@DRONE_SANTANA,t,InitSTATE,...                            %TRAJECTORY, ControllerStruct, gamma);
%                                  SimStruct);
% ERROR = ode4(@ERROR_Fuzzy,t,InitERROR,...
%                                 SimStruct);  



[t,STATE] = ode45(@DRONE_SANTANA,t,InitSTATE,[],...                            %TRAJECTORY, ControllerStruct, gamma);
                                 SimStruct);
[t,ERROR] = ode45(@ERROR_Fuzzy,t,InitERROR,[],...                            %TRAJECTORY, ControllerStruct, gamma);
                                 SimStruct);


%% OUTPUT PLOTTING
gamma
InitSTATE
simStep
Tfinal
TRAJECTORY
SimStruct.controller
SimStruct.sys.sat

Output;

Check_FuzzyTracking;
