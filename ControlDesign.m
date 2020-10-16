[A,B,h] = ErrorModeling(Modeltype,gamma);

switch ControlType
    case 'openloop'
        K = zeros(4);
        disp('Virtual controller set to open loop');
        
    case 'MozelliTeo6'
        fi=0.01*ones(size(A,2));
        mu=0.1;
        
        if(Modeltype == 2)
            K = LMI_Teo6MozelliMOD(A,B,fi,mu);
        else
            K = LMI_Teo6Mozelli(A,B,fi,mu);
        end
        ControlType = 'PDC';
    otherwise
        K=[];
        disp('The controller you chose is not an option!')
end

SimStruct.sys.sat = Saturation;

SimStruct.controller.type = ControlType;
SimStruct.controller.model.type = Modeltype;
SimStruct.controller.model.A = A;
SimStruct.controller.model.B = B;
SimStruct.controller.model.h = h;
SimStruct.controller.gain    = K;

%% auxiliar functions
function F = LMI_Teo6Mozelli(A,B,fi,mu)
% This implements Theorem 6 on
% "A systematic approach to improve multiple Lyapunov function stability
% and stabilization conditions for fuzzy systems" by
% L. A. Mozelli, R. M. Palhares, G. S. C. Avellar. 2009
% -------------------------------------------------------------------
r = size(B,2);
nA = size(A{1},2);
mB = size(B{1},2);

T = cell(1,r);
Y = sdpvar(nA,nA,'symmetric');
R = sdpvar(nA,nA,'full');
S = cell(1,r);

Tfi = 0;
for i = 1:r
    T{i} = sdpvar(nA,nA,'symmetric');
    S{i} = sdpvar(nA,mB);
    Tfi = Tfi + fi(i)*(T{i}+Y);
end

Csi = cell(r);
Csibarra = cell(r);
for i = 1:r
    for j = 1:r
        star = T{i} - mu*(A{i}*R'-B{i}*S{i}') + R;
        Csi{i,j} = [Tfi-A{i}*R'-R*A{i}'+B{i}*S{i}'+S{i}*B{i}'  star';
            star                            mu*(R+R')];
    end
end

for i = 1:r
    for j = 1:r
        Csibarra{i,j} = Csi{i,j} + Csi{j,i};
    end
end

Restr = [];
for j = 1:r
    for i = 1:j
        Restr = [Restr T{i}>=0 ...
            T{i}+Y>=0 ...
            Csi{i,i} <=0 ...
            Csibarra{i,j}<=0];
    end
end
% Configurando o Solver.
opts=sdpsettings;
opts.verbose=0;

% Resolvendo as LMIs
sol = solvesdp(Restr,[],opts);

% Verifica se as LMI's sao factiveis
p=min(checkset(Restr));
if (p > 0)
    R = double(R);
    Y = double(Y);
    F = cell(1,r);
    for i = 1:r
        T{i} = double(T{i});
        S{i} = double(S{i});
        F{i} = S{i}'/(R');
    end
else
    F=[];
    disp('LMIs Infactiveis')
end
end

function K = LMI_Teo6MozelliMOD(A,B,fi,mu)
% Esta fun��o encontra condi��es suficientes para o projeto de
% controladores fuzzy, considerando realimenta��o de estados.
% Esse trabalho implementa o resultado do Teorema 6 de:
% A systematic approach to improve multiple Lyapunov function stability
% and stabilization conditions for fuzzy systems
% L. A. Mozelli, R. M. Palhares, G. S. C. Avellar. 2009
% -------------------------------------------------------------------
ri = size(B,2);
nA = size(A{1,1},2);     % Determina a dimens�o da matriz A
mB = size(B{1,1},2);     % Determina o n�mero de colunas de B
% Cell que vai armazenar os controladores locais
T = cell(1,ri);
S = cell(1,ri);
% Criando as vari�veis matriciais
Y = sdpvar(nA,nA,'symmetric');  % Declaracao de Y nxn simetrica
R = sdpvar(nA,nA,'full');       % Declaracao de R nxn full
% Criando as vari�veis Ti, Si e Tfi
Tfi = 0;
Restr = [];
for t1 = 1:ri
	T{1,t1} = sdpvar(nA,nA,'symmetric');  % Declaracao de Ti nxn simetrica
    S{1,t1} = sdpvar(nA,mB);              % Declaracao de Si nxm
    Tfi = Tfi + fi(1,t1)*(T{1,t1}+Y);
end
% % Criando o conjunto de restri��es LMI
for t1 = 1:ri   
    Restr = [Restr T{1,t1}>=0 T{1,t1}+Y>=0 ...
[Tfi-A{t1,t1}*R'-R*A{t1,t1}'+B{1,t1}*S{1,t1}'+S{1,t1}*B{1,t1}'  ...
(T{1,t1}-mu*(A{t1,t1}*R'-B{1,t1}*S{1,t1}')+R)';
      T{1,t1}-mu*(A{t1,t1}*R'-B{1,t1}*S{1,t1}')+R  mu*(R+R')]<=0];
    for t2 = t1+1:ri
    Restr = [Restr ...
[Tfi-A{t1,t2}*R'-R*A{t1,t2}'+B{1,t1}*S{1,t2}'+S{1,t2}*B{1,t1}'  ...
(T{1,t1}-mu*(A{t1,t2}*R'-B{1,t1}*S{1,t2}')+R)';
      T{1,t1}-mu*(A{t1,t2}*R'-B{1,t1}*S{1,t2}')+R  mu*(R+R')]+...
[Tfi-A{t2,t1}*R'-R*A{t2,t1}'+B{1,t2}*S{1,t1}'+S{1,t1}*B{1,t2}'  ...
(T{1,t2}-mu*(A{t2,t1}*R'-B{1,t2}*S{1,t1}')+R)';
      T{1,t2}-mu*(A{t2,t1}*R'-B{1,t2}*S{1,t1}')+R  mu*(R+R')]<=0];
    end
end
% Configurando o Solver.
opts=sdpsettings;
% opts.solver='lmilab';
opts.verbose=0;
% Resolvendo as LMIs
sol = solvesdp(Restr,[],opts);
% Verifica se as LMI's sao factiveis
p=min(checkset(Restr));
if p > 0
    R = double(R);
    Y = double(Y);
    K = cell(1,ri);
    for t1 = 1:ri,
        T{1,t1} = double(T{1,t1});
        S{1,t1} = double(S{1,t1});
        K{1,t1} = S{1,t1}'*inv(R');
    end
else
    K=[];
    display('LMIs Infactiveis')
end
end