% Exemplo drone
clear all
clc
close all

%% FUZZY SYSTEM

load gamma
g1=gamma(1);g2=gamma(2);g3=gamma(3);g4=gamma(4);
g5=gamma(5);g6=gamma(6);g7=gamma(7);g8=gamma(8);

bound=1;
z10=-bound; % psi=pi
z11=bound;  % psi=0 ou pi
% z2(psi)= sen(psi)
z20=-bound; % psi=3pi/2
z21=bound;  % psi= pi/2

% Non linearities, local models and order of system
p=4;    r=2^p; ri=4;

% Criando as cell para os dois extremos da incerteza
M=cell(1,ri);
N=cell(1,ri);
R=cell(1,ri); 

% -------------------------------------------------
% Gerando os modelos locais para o caso da incerteza 1
% Neste caso foram considerados os valores de gamma originais

%      M = [gamma(1)*cos(psi), -gamma(3)*sin(psi),      0,        0;
%           gamma(1)*sin(psi),  gamma(3)*cos(psi),      0,        0;
%                 0,                  0,              gamma(5),   0;
%                 0,                  0,                0,    gamma(7)];
M{1,1}=[g1*z10 -g3*z20   0   0
        g1*z20  g3*z10   0   0
             0     0    g5   0
             0     0     0   g7];
M{1,2}=[g1*z10 -g3*z21   0   0
        g1*z21  g3*z10   0   0
             0     0    g5   0
             0     0     0   g7];
M{1,3}=[g1*z11 -g3*z20   0   0
        g1*z20  g3*z11   0   0
             0     0    g5   0
             0     0     0   g7];
M{1,4}=[g1*z11 -g3*z21   0   0
        g1*z21  g3*z11   0   0
             0     0    g5   0
             0     0     0   g7];
% ********************************
%     N = [ g2*cos(psi) -g4*sin(psi)   0   0
%           g2*sin(psi)  g4*cos(psi)   0   0
%                 0               0   g6   0
%                 0               0    0   g8]
N{1,1}=[g2*z10 -g4*z20  0  0
        g2*z20  g4*z10  0  0
           0       0   g6  0
           0       0    0 g8];
N{1,2}=[g2*z10 -g4*z21  0  0
        g2*z21  g4*z10  0  0
           0       0   g6  0
           0       0    0 g8];
N{1,3}=[g2*z11 -g4*z20  0  0
        g2*z20  g4*z11  0  0
           0       0   g6  0
           0       0    0 g8];
N{1,4}=[g2*z11 -g4*z21  0  0
        g2*z21  g4*z11  0  0
           0       0   g6  0
           0       0    0 g8];
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
% -------------------------------------------------
% celldisp(M)
% celldisp(N)
% celldisp(R)

[LN, CN]=size(N{1,1});
    A = cell(1,r);
    tj=1;
for t1 = 1:ri    
    for t2 = 1:ri
        A{1,tj} = [-N{1,t1}*(R{1,t2}')  zeros(LN,CN)
                     eye(LN)          zeros(LN,CN)];
                 
        B{1,tj} = [eye(4);zeros(4)];
        C{1,tj} = [zeros(4) eye(4)];
        if rank(C{1,tj}*ctrb(A{1,tj},B{1,tj})) < 4
            disp('Nao controlavel: ')
            disp(tj)
        end 
        tj=tj+1;
    end
end