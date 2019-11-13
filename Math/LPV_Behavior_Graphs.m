% This script generates the graphs for my QUALI, where it shows 
% the nonminimum phase behavior and instability caused by the 
% continuous change in parameter


% LPV system with non minimum phase behavior

clc
clear all
close all

% syms rho t
% x=sym('x',[3 1]);
t = 0:0.1:10;

rho = cos(2*t);
u = exp(t)*sin(t);
x0 = [0 0 0]';

% A = [0, (2-rho)^2, 1 + 0.5*rho + (2-rho)^2;
%      1,         0,              0.2;
%      0,0,0];
% B = [0 0 1]';
C = [0 1 1];

[t,x] = ode45(@nonmin_phase(t,x,rho,u),t,x0))
