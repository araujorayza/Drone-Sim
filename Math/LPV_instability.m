clc;
clear all;

time=0:0.1:7;

dx = @(t,x) [                0,   1;
              - cos(2*t)/2 - 1, 1/5]*x;
          
rng('shuffle');       
xin = -1 + 2*rand([2 7])
Y = cell(length(xin));
    
for i=1:length(xin)
    x0=xin(:,i);
    [t,y] = ode45(dx,time,x0);
    Y{i}=y;
end 

h=figure; hold on;
for i=1:length(xin)
    plot(xin(1,i),xin(2,i),'*',Y{i}(:,1),Y{i}(:,2));
    legend('Initial Position','Trajectory')
end 
grid on;
print(h,'LPV_instability','-dpdf')