clear all
close all
clc
load 2step.mat

L = SereniTeo2(A,B,C,15);

simStep = 1;
Tfinal  = 6;


options = odeset('RelTol',1e-9,'AbsTol',1e-9);
[t,STATE] = ode45(@(t,y) DRONE_SANTANA(t,y,SimStruct),t,InitSTATE,options);
[t,ERROR] = ode45(@(t,y) ERROR_Fuzzy(t,y,SimStruct),t,InitERROR,options);
                                 

%%%%%%%%%%%%%%%%%TRAJECTORY PLOT%%%%%%%%%%%%%%%%%%%%%%%
figure(1);

hold on;
plot(STATE(:,PosX),STATE(:,PosY),'*');
plot(ERROR(:,PosX)+ DES_STATE(:,PosX),ERROR(:,PosY) + DES_STATE(:,PosY),'rd');
plot(DES_STATE(:,PosX),DES_STATE(:,PosY),'k*');
hold off
grid on
axis equal
legend('Trajectory')
legend('NonLinSys','FuzzyErrorSys','Ref')




function L = SereniTeo2(A,B,C,gamma)
N = size(A,2);
nA = size(A{1,1},2);
mB = size(B{1,1},2);
mC = size(C,1);

F = cell(1,N);
G = cell(1,N);
J = cell(1,N);
P = sdpvar(nA,nA,'symmetric');
H = sdpvar(mB,mB,'full');
for i=1:N
    F{i} = sdpvar(nA,nA,'full');
    G{i} = sdpvar(nA,nA,'full');
    J{i} = sdpvar(mB,mC,'full');
end
K = Quadratic(A,B);
LMIs = [P>0];
for i=1:N
    a11 = A{i}'*F{i}' + F{i}*A{i} + K'*B{i}'*F{i}' + F{i}*B{i}*K + 2*gamma*P;
    a21 = P - F{i}' + G{i}*A{i} + G{i}*B{i}*K;
    a31 = B{i}'*F{i}' + J{i}*C - H*K;
    a22 = -G{i}-G{i}';
    a32 = B{i}'*G{i}';
    a33 = -H-H';
    
    LMIs = [LMIs,   [a11 a21' a31';
                    a21 a22  a32';
                    a31 a32  a33]];
    for j = i+1:N
        a11 = A{i}'*F{j}' + K'*B{i}'*F{j}' + A{j}'*F{i}' + K'*B{j}'*F{i}';
        a11 = a11 + a11' + 4*gamma*P;
        
        a21 = 2*P - F{i}' -F{j}' + G{i}*A{j} + G{i}*B{j}*K + G{j}*A{i} + G{j}*B{i}*K;
        a31 = B{i}'*F{j}' + J{j}*C + J{i}*C + B{j}'*F{i}' - 2*H*K;
        a22 = -G{i}-G{i}'-G{j}-G{j}';
        a32 = B{i}'*G{j}' + B{j}'*G{i}';
        a33 = -2*H-2*H';
        
        LMIs = [LMIs,[  a11 a21' a31';
                        a21 a22  a32';
                        a31 a32  a33]];
    end
end
opts=sdpsettings;
opts.solver = 'lmilab';
sol = optimize(LMIs,[],opts);
che=min(check(LMIs));
if che > 0
    disp('funcionou!')
    H = value(H);
    for i=1:N
        J{i} = value(J{i});
        P  = value(P);
        L{i} = inv(H)*J{i};
    end
    
else
    L=[];
    disp('nao funcionou')
end
end

function K = Quadratic(A,B)
beta = 1;
N = size(A,2);
nA = size(A{1,1},2);
mB = size(B{1,1},2);
W = sdpvar(nA,nA,'symmetric');
Z = sdpvar(mB,nA,'full');

LMIs = [W>0];

for i=1:N
    LMIs = [LMIs, A{i}*W + W*A{i}' + B{i}*Z + Z'*B{i}' + 2*beta*W < 0];
end
opts=sdpsettings;
opts.solver = 'lmilab';
sol = optimize(LMIs,[],opts);
che=min(check(LMIs));
if che > 0
    disp('quadratico funcionou')
    W = value(W);
    K = value(Z)*inv(W)
else
    K = []
    error('quadratico nao funcionou')  
end
end