clc; clear all; close all;
NonlinearMatrices
syms psi real

A_NL = [-N(psi)*R(psi)' zeros(4);
             eye(4)     zeros(4)];
%% ------------------------------------------------------------------------
% Now we need to calculate the fuzzy model completely symbolic
z = {@(Psi) cos(Psi);
     @(Psi) sin(Psi)};

interval = 0:0.01:2*pi;

mini=1;maxi=2;
Z=sym('Z',[2 2]);
syms z1 z2

%calculate the max and min value of z in interval
%and store it in the Z matrix
for i=1:length(z)
    Z(i,maxi) = round(max(z{1}(interval)),2);
    Z(i,mini) = round(min(z{1}(interval)),2);
end

%Substitute nonlinearities
% N = subs(N(psi),z{1}(psi),z1);
% N = subs(N,z{2}(psi),z2);
% 
% 
% R = subs(R(psi),z{1}(psi),z1);
% R = subs(R,z{2}(psi),z2);
% 
% 
% M = subs(M(psi),z{1}(psi),z1);
% M = subs(M,z{2}(psi),z2);

% calculate local models
% Nummodels = 2^(length(z);
% Mask      = str2num(dec2bin(Nummodels-1));
% for i=1:Nummodels
%     access = i - 1;
%     access = str2num(dec2bin(access))
%     access = access + Mask
%     access = int2str(modelNum)-'0'; %separate the bits in a vector
%     
%     N{i}= subs(N,[z1 z2],[Z() Z()])
%     M{i}=
%     R{i}=
%     
% end 
    z10 = -1; 
    z11 =  1;  
    z20 = -1; 
    z21 =  1;  
    
    ri=4;
    M=cell(1,ri);
    N=cell(1,ri);
    R=cell(1,ri);

%      M = [gamma(1)*cos(psi), -gamma(3)*sin(psi),      0,        0;
%           gamma(1)*sin(psi),  gamma(3)*cos(psi),      0,        0;
%                 0,                  0,              gamma(5),   0;
%                 0,                  0,                0,    gamma(7)];
    M{1,1}=[gamma(1)*z10 -gamma(3)*z20   0      0
            gamma(1)*z20  gamma(3)*z10   0      0
                 0          0       gamma(5)    0
                 0          0           0   gamma(7)];
    M{1,2}=[gamma(1)*z10 -gamma(3)*z21   0   0
            gamma(1)*z21  gamma(3)*z10   0   0
                 0     0    gamma(5)   0
                 0     0     0   gamma(7)];
    M{1,3}=[gamma(1)*z11 -gamma(3)*z20   0   0
            gamma(1)*z20  gamma(3)*z11   0   0
                 0     0    gamma(5)   0
                 0     0     0   gamma(7)];
    M{1,4}=[gamma(1)*z11 -gamma(3)*z21   0   0
            gamma(1)*z21  gamma(3)*z11   0   0
                 0     0    gamma(5)   0
                 0     0     0   gamma(7)];
% ********************************
%     N = [ gamma(2)*cos(psi) -gamma(4)*sin(psi)   0   0
%           gamma(2)*sin(psi)  gamma(4)*cos(psi)   0   0
%                 0               0   gamma(6)   0
%                 0               0    0   gamma(8)]
    N{1,1}=[gamma(2)*z10 -gamma(4)*z20  0  0
            gamma(2)*z20  gamma(4)*z10  0  0
               0       0   gamma(6)  0
               0       0    0 gamma(8)];
    N{1,2}=[gamma(2)*z10 -gamma(4)*z21  0  0
            gamma(2)*z21  gamma(4)*z10  0  0
               0       0   gamma(6)  0
               0       0    0 gamma(8)];
    N{1,3}=[gamma(2)*z11 -gamma(4)*z20  0  0
            gamma(2)*z20  gamma(4)*z11  0  0
               0       0   gamma(6)  0
               0       0    0 gamma(8)];
    N{1,4}=[gamma(2)*z11 -gamma(4)*z21  0  0
            gamma(2)*z21  gamma(4)*z11  0  0
               0       0   gamma(6)  0
               0       0    0 gamma(8)];
% ********************************
%     R = [ cos(psi),    -sin(psi),      0,    0;
%           sin(psi),     cos(psi),      0,    0;
%                 0,          0,         1,    0;
%                 0,          0,         0,    1];
    R{1,1}=[z10 -z20  0  0
            z20  z10  0  0
             0    0   1  0
             0    0   0  1];
    R{1,2}=[z10 -z21  0  0
            z21  z10  0  0
             0    0   1  0
             0    0   0  1];
    R{1,3}=[z11 -z20  0  0
            z20  z11  0  0
             0    0   1  0
             0    0   0  1];
    R{1,4}=[z11 -z21  0  0
            z21  z11  0  0
             0    0   1  0
             0    0   0  1];

 H=sym('H',[length(z) 2]);

for i=1:length(z)
    H(i,mini) = (z{i}(psi)-Z(i,mini))/(Z(i,maxi)-Z(i,mini)); %this is inverted like this.
    H(i,maxi) = 1 - H(i,mini);
end

h=CalcDefuzzWeights(H);
         
Nh     = 0*N{1};
Rh     = 0*R{1};
         
for k=1:length(h)
    Nh=Nh+h(k)*N{k};
    Rh=Rh+h(k)*R{k};
end
         
A_FZ = [-Nh*Rh' zeros(4);eye(4) zeros(4)];
simplify(A_FZ)
simplify(A_NL)
simplify(A_FZ-A_NL)
%show scheduling parameters as in sum(i)sum(j)[-NiRjt ...] = sum(k)rho_kA_k
%Obtain the combinations of membership functions h_i, 2 by 2
% h1^2, h1h2, h1h3, h1h4,...
% the order of bits vs functions is: h1 h2 h3 h4
rho=sym('rho',[16 1]);
for i=1:length(rho)                 % h1 x h1
    index=1;                        % h1 x h2 ...
    for k=1:length(h)               % h2 x h2
        for j=1:length(h)           % h2 x h3 ..
            rho(index) = h(k)*h(j); % h3 x h4
            index = index + 1;      % h4 x h4
        end 
    end 
end 
unique(simplify(rho))