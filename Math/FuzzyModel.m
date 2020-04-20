clc;
clear all;
close all;
syms t psi(t) dpsi
assume(t,'real');

MaxRotSpd = 100;


min=1;max=2;

z = {@(Psi) cos(Psi);
     @(Psi) sin(Psi)};

for i=1:length(z)
             %min   max
    Z(i,:) = [-1     1];
end
 
for i=1:length(z)
    memb(i,1) = (z{i}(psi)-Z(i,min))/(Z(i,max)-Z(i,min));
    memb(i,2) = 1 - memb(i,1);
end

h = CalcDefuzzWeights(memb);

syms x y
h_dot = diff(h,t)
h_dot = subs(h_dot,{psi,diff(psi(t), t)},{x,y})

for i=1:length(h_dot)
    figure;
    fsurf(h_dot(i),[0 2*pi -MaxRotSpd*pi/180 MaxRotSpd*pi/180])
end 
