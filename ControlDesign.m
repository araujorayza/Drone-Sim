if(Modeltype == 1)
    [A,B,h] = CalcTSModel(gamma);
elseif(Modeltype == 2)
    
end

switch ControlType
    case 'StateFeedback'
        psi=InitSTATE(8);
        K=ControlDesign_LinearStateFeedback(psi);
        
    case 'Fuzzy'
        fi=0.01*ones(size(A,2));
        mu=0.1;
        in=3;

        K = LMI_Teo51 (A,B,fi,mu,in);  
    case 'LQR'
        psi=InitSTATE(8);
        K=ControlDesign_LQR(psi);
    case 'FeedbackLin'
        K=cell(2,1);
        K{1}=10*eye(4);
        K{2}=10*eye(4); 
    case 'openloop'
        K = zeros(4);
        disp('Controller set to open loop')
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
function K = ControlDesign_LinearStateFeedback(psi)
    load gamma gamma
    N = [ gamma(2)*cos(psi), -gamma(4)*sin(psi),      0,        0;
        gamma(2)*sin(psi),  gamma(4)*cos(psi),      0,        0;
        0,                  0,              gamma(6),   0;
        0,                  0,                0,    gamma(8)];
    clear gamma
    R = [ cos(psi),    -sin(psi),      0,    0;
        sin(psi),     cos(psi),      0,    0;
        0,          0,         1,    0;
        0,          0,         0,    1];

    n=length(N);
    %dE = A*E + B*U
    A=[-N*R' zeros(n);
        eye(n) zeros(n)];
    B=[eye(n);zeros(n)];

    P=-1:-1:-8;
    K=place(A,B,P);
end 

function K = LMI_Teo51 (A,B,fi,mu,in)
% TEOREMA 5.1 DO NOSSO ARTIGO
% Esta fun��o encontra condi��es suficientes para o projeto de
% controladores fuzzy, considerando realimenta��o de estados.
% Esse trabalho implementa uma "relaxa��o" do resultado do Teorema 6 de:
% A systematic approach to improve multiple Lyapunov function stability
% and stabilization conditions for fuzzy systems
% L. A. Mozelli, R. M. Palhares, G. S. C. Avellar. 2009
% -------------------------------------------------------------------

ri = size(A,2);       % Determina o n�mero de modelos locais do sistema
nA = size(A{1,1},2);  % Determina a dimens�o da matriz A
mB = size(B{1,1},2);  % Determina o n�mero de colunas de B
% Cell que vai armazenar os controladores locais
T = cell(1,ri);
S = cell(1,ri);
% Criando as vari�veis matriciais
Y = sdpvar(nA,nA,'symmetric');      % Declaracao de Y nxn simetrica
R = sdpvar(nA,nA,'full');      % Declaracao de R nxn full
% Criando as vari�veis Ti, Si e Tfi
Tfi1 =  fi(in,1)*Y;
Tfi2 = -fi(in,1)*Y;
% Criando as vari�veis definidas positivas Ti e as vari�veis Si
Restr = [];
for t1 = 1:ri,
	T{1,t1} = sdpvar(nA,nA,'symmetric');  % Declaracao de Ti nxn simetrica
    S{1,t1} = sdpvar(nA,mB);              % Declaracao de Si nxm
    Restr = [Restr T{1,t1}>=0];
end
% Criando a vari�vel Tfi
for t1 = 1:ri,
    if t1~=in
       Tfi1 = Tfi1 + fi(t1,1)*(T{1,t1}-T{1,in}+Y);
       Tfi2 = Tfi2 + fi(t1,1)*(T{1,t1}-T{1,in}+Y);
       Restr = [Restr T{1,t1}-T{1,in}+Y>=0];
    end
end
% % Criando o conjunto de restri��es LMI
for t1 = 1:ri,    
    Restr = [Restr  ...
[Tfi1-A{1,t1}*R'-R*A{1,t1}'+B{1,t1}*S{1,t1}'+S{1,t1}*B{1,t1}'  ...
     (T{1,t1}-mu*(A{1,t1}*R'-B{1,t1}*S{1,t1}')+R)';
      T{1,t1}-mu*(A{1,t1}*R'-B{1,t1}*S{1,t1}')+R  mu*(R+R')]<=0 ...
[Tfi2-A{1,t1}*R'-R*A{1,t1}'+B{1,t1}*S{1,t1}'+S{1,t1}*B{1,t1}'  ...
     (T{1,t1}-mu*(A{1,t1}*R'-B{1,t1}*S{1,t1}')+R)';
      T{1,t1}-mu*(A{1,t1}*R'-B{1,t1}*S{1,t1}')+R  mu*(R+R')]<=0];
    for t2 = t1+1:ri,
    Restr = [Restr  ...
[Tfi1-A{1,t1}*R'-R*A{1,t1}'+B{1,t1}*S{1,t2}'+S{1,t2}*B{1,t1}'  ...
     (T{1,t1}-mu*(A{1,t1}*R'-B{1,t1}*S{1,t2}')+R)';
      T{1,t1}-mu*(A{1,t1}*R'-B{1,t1}*S{1,t2}')+R  mu*(R+R')]+...
    [Tfi1-A{1,t2}*R'-R*A{1,t2}'+B{1,t2}*S{1,t1}'+S{1,t1}*B{1,t2}'  ...
     (T{1,t2}-mu*(A{1,t2}*R'-B{1,t2}*S{1,t1}')+R)';
      T{1,t2}-mu*(A{1,t2}*R'-B{1,t2}*S{1,t1}')+R  mu*(R+R')]<=0 ...
[Tfi2-A{1,t1}*R'-R*A{1,t1}'+B{1,t1}*S{1,t2}'+S{1,t2}*B{1,t1}'  ...
     (T{1,t1}-mu*(A{1,t1}*R'-B{1,t1}*S{1,t2}')+R)';
      T{1,t1}-mu*(A{1,t1}*R'-B{1,t1}*S{1,t2}')+R  mu*(R+R')]+...
    [Tfi2-A{1,t2}*R'-R*A{1,t2}'+B{1,t2}*S{1,t1}'+S{1,t1}*B{1,t2}'  ...
     (T{1,t2}-mu*(A{1,t2}*R'-B{1,t2}*S{1,t1}')+R)';
      T{1,t2}-mu*(A{1,t2}*R'-B{1,t2}*S{1,t1}')+R  mu*(R+R')]<=0];
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
    K = cell(ri,1);
    for t1 = 1:ri,
        T{1,t1} = double(T{1,t1});
        S{1,t1} = double(S{1,t1});
        K{t1,1} = S{1,t1}'*inv(R');
    end
%     celldisp(T)
%     celldisp(S)
else
    K=[];
    disp('O sistema n�o pode ser controlado')
end
end 

function [K] = ControlDesign_LQR(psi)
    load gamma gamma
    N = [ gamma(2)*cos(psi), -gamma(4)*sin(psi),      0,        0;
          gamma(2)*sin(psi),  gamma(4)*cos(psi),      0,        0;
            0,                  0,              gamma(6),       0;
            0,                  0,                0,    gamma(8)];
    clear gamma
    R = [ cos(psi),    -sin(psi),      0,    0;
        sin(psi),     cos(psi),      0,    0;
        0,          0,         1,    0;
        0,          0,         0,    1];

    n=length(N);
    %dE = A*E + B*U
    A=[-N*R' zeros(n);
        eye(n) zeros(n)];
    B=[eye(n);zeros(n)];
    
    sys=ss(A,B,eye(2*n),0);
    R=10*eye(2*n);
    Q=eye(4);
    
    [K,~,~]=lqr(sys,R,Q,0);
end

