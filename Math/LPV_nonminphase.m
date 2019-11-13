% % This script generates the graphs for my QUALI, where it shows 
% % the nonminimum phase behavior and instability caused by the 
% % continuous change in parameter
% 
% clc
% clear all
% close all
% 
% syms rho s
% 
% A = [0 , (2-rho)^2 , 1+0.5*rho+(2-rho)^2;
%         1 ,  0 , 0.2;
%             0 ,  0  , 0]
% B = [0;0;1];
% 
% C = [0 1 1];
% 
% G=C*inv(s*eye(3)-A)*B;
% G=expand(G)
% 
% % ZEROES
% %  - (- 2*rho - 99/25)^(1/2)/2 - 1/10
% %    (- 2*rho - 99/25)^(1/2)/2 - 1/10
% % POLES
% %  0
% %  2 - rho
% %  rho - 2

%% 
clc
clear all
close all

t = 0:0.1:50;


x0 = [0 0 0]';

[t,x] = ode45(@nonmin_phase,t,x0);
plot(x(:,2)+x(:,3))