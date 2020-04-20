clc;
close all;
clear all;

t=0:0.01:30;


W_static = 1;       
time=[t(1) t(end)];


for i=1:length(t)
    W(:,i)=WindDistGust(t(i),W_static,time);
end 

plot(t,W)