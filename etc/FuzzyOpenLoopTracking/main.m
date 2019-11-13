clc
clear all
close all


Einit = zeros(8,1);
t=0:0.1:0.5;

E_nonLin = ode4(@model_nonlinear,t,Einit);
E_fuzzy  = ode4(@model_fuzzy,t,Einit);


plot(t,E_nonLin(:,1),'*')
hold on;
plot(t,E_fuzzy(:,1),'d')