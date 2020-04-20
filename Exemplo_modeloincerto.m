clear all
clc
close all

%% FUZZY SYSTEM
% load gamma.mat

% Valores enviados pela Raiza por email
gamma = [4.5752,0.1149,5.2296,0.2364,4.4206,3.1341,5.9289,-0.3868];
g1=gamma(1);g2=gamma(2);g3=gamma(3);g4=gamma(4);
g5=gamma(5);g6=gamma(6);g7=gamma(7);g8=gamma(8);

bound=1;
z10=-bound; % psi=pi
z11=bound;  % psi=0 ou pi
% z2(psi)= sen(psi)
z20=-bound; % psi=3pi/2
z21=bound;  % psi= pi/2

ri=4;
% Criando as cell para os dois extremos da incerteza
M=cell(2,ri);
N=cell(2,ri);
R=cell(1,ri); % A matriz R nï¿½o depende das incertezas
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
% Gerando os modelos locais para o caso da incerteza 2
% Neste caso foi considerado um acrï¿½scimo de 80% do valor original do gama
% Ou seja gama < incerteza < 1.8*gama
aumento=1.8;
gamma = aumento*[4.5752,0.1149,5.2296,0.2364,4.4206,3.1341,5.9289,-0.3868];
g1=gamma(1);g2=gamma(2);g3=gamma(3);g4=gamma(4);
g5=gamma(5);g6=gamma(6);g7=gamma(7);g8=gamma(8);
% ********************************
M{2,1}=[g1*z10 -g3*z20   0   0
        g1*z20  g3*z10   0   0
             0     0    g5   0
             0     0     0   g7];
M{2,2}=[g1*z10 -g3*z21   0   0
        g1*z21  g3*z10   0   0
             0     0    g5   0
             0     0     0   g7];
M{2,3}=[g1*z11 -g3*z20   0   0
        g1*z20  g3*z11   0   0
             0     0    g5   0
             0     0     0   g7];
M{2,4}=[g1*z11 -g3*z21   0   0
        g1*z21  g3*z11   0   0
             0     0    g5   0
             0     0     0   g7];
% ********************************
N{2,1}=[g2*z10 -g4*z20  0  0
        g2*z20  g4*z10  0  0
           0       0   g6  0
           0       0    0 g8];
N{2,2}=[g2*z10 -g4*z21  0  0
        g2*z21  g4*z10  0  0
           0       0   g6  0
           0       0    0 g8];
N{2,3}=[g2*z11 -g4*z20  0  0
        g2*z20  g4*z11  0  0
           0       0   g6  0
           0       0    0 g8];
N{2,4}=[g2*z11 -g4*z21  0  0
        g2*z21  g4*z11  0  0
           0       0   g6  0
           0       0    0 g8];
% -------------------------------------------------
celldisp(M)
celldisp(N)
celldisp(R)

raio=5;
save matrizes_fuzzy N M R z10 z11 z20 z21 raio g1 g2 g3 g4 g5 g6 g7 g8

% Valor inicial para a taxa de variaï¿½ï¿½o mï¿½xima das funï¿½ï¿½es de pertinï¿½ncia
V0=5;  % | hponto | <= V0
%fi=[V0 aumento*V0]; mu = 0.1;
fi=[V0 V0]; mu = 0.1;
% Total variation matrix
p=log2(ri);
MT=function_total_matrix (p);

% --------------------------------------------------
%% PROJETO CONSIDERANDO ESTABILIZAï¿½ï¿½O DO MODELO DO ERRO
[Q,W,K] = lmi_estabilizacao (N,R,mu,fi,MT);
display('Controlador para estabilização')
celldisp(K)

% --------------------------------------------------
%% PROJETO CONSIDERANDO ESTABILIZAï¿½ï¿½O + NORMA HInf DO MODELO DO ERRO
C=[0 0 0 0 1 0 0 0
   0 0 0 0 0 1 0 0
   0 0 0 0 0 0 1 0
   0 0 0 0 0 0 0 1];
ga=0.1; % Parâmetro usado para minimizar a atenuação
[Q,W,KI] = lmi_Hinf (N,R,C,mu,fi,MT,ga);
display('Controlador para atenuação de disturbio')
celldisp(K)

 A = cell(2,ri,ri);
B1 = cell(2,ri,ri);
B2 = cell(1,ri);
[LN,CN]=size(N{1,1});
for t1 = 1:ri,
    B2{1,t1} = [eye(LN);zeros(CN)];
    for t2 = 1:ri,
        % Incerteza 1
        A{1,t1,t2} = [-N{1,t1}*(R{1,t2}')  zeros(LN,CN)
                          eye(LN)          zeros(LN,CN)];
        B1{1,t1,t2} = [-eye(LN)       -N{1,t1}*(R{1,t2}')
                       zeros(LN,CN)   zeros(LN,CN)];
        % Incerteza 2
        A{2,t1,t2} = [-N{2,t1}*(R{1,t2}')  zeros(LN,CN)
                          eye(LN)          zeros(LN,CN)];
        B1{2,t1,t2} = [-eye(LN)       -N{2,t1}*(R{1,t2}')
                       zeros(LN,CN)   zeros(LN,CN)];
    end
end

for t1 = 1:ri,
    for t2 = 1:ri,
        if t1==t2
            AF=A{1,t1,t1}-B2{1,t1}*KI{1,t1};
            AF=max(real(eig(AF)));
            fprintf('\n EIG(A_1%d%d) =  ',t1,t2)
            fprintf(' %.5f ',AF')
            AF=A{2,t1,t1}-B2{1,t1}*KI{1,t1};
            AF=max(real(eig(AF)));
            fprintf('\n EIG(A_2%d%d) =  ',t1,t2)
            fprintf(' %.5f ',AF')
        else
            AF=A{1,t1,t2}-B2{1,t1}*KI{1,t2}+A{1,t2,t1}-B2{1,t2}*KI{1,t1};
            AF=max(real(eig(AF)));
            fprintf('\n EIG(A_1%d%d+A_1%d%d) =  ',t1,t2,t2,t1)
            fprintf(' %.5f ',AF')
            AF=A{2,t1,t2}-B2{1,t1}*KI{1,t2}+A{2,t2,t1}-B2{1,t2}*KI{1,t1};
            AF=max(real(eig(AF)));
            fprintf('\n EIG(A_2%d%d+A_2%d%d) =  ',t1,t2,t2,t1)
            fprintf(' %.5f',AF')
        end
    end
end
fprintf('\n\n')

% save matrizes_fuzzy N M R K z10 z11 z20 z21 raio
% % Simulando o sistema controlado
% xg=[0;0;0;0;0;1;1.2;1.5];
% T=0:0.1:40;
% [T,xe]=ode45('DRONE_controlado2019',T,xg);
% figure(3)
% plot(T,xe(:,5:8))
% legend('x5','x6','x7','x8')
% 
% figure(4)
% plot(xe(:,5),xe(:,6))
% axis('equal')
