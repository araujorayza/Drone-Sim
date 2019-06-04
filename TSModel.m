ri=4; % # of local models
min=1;
max=2;


z{1}=@(psi) cos(psi);
z{2}=@(psi) sin(psi);


Z(1,min)=-1;
Z(1,max)=1;

Z(2,min)=-1;
Z(2,max)=1;

          
z10=Z(1,min);
z20=Z(2,min);
z11=Z(1,max);
z21=Z(2,max);

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


A=cell(1,ri);
B=cell(1,ri);
n=size(N{ri},1);

for i=1:ri
    A{1,i}=[-N{i}*R{i}' zeros(n);
                eye(n)  zeros(n)];
    B{1,i}=[eye(4);
            zeros(n)];
end