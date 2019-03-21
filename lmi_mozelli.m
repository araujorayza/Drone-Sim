function K = lmi_mozelli (M,N,R,fi,mu)
% Esta função encontra condições suficientes para o projeto de
% controladores fuzzy, considerando realimentação de estados.
% Esse trabalho implementa o resultado do Teorema 6 de:
% A systematic approach to improve multiple Lyapunov function stability
% and stabilization conditions for fuzzy systems
% L. A. Mozelli, R. M. Palhares, G. S. C. Avellar. 2009
% -------------------------------------------------------------------
ri = size(M,2);
 A = cell(ri,ri);
 B = cell(1,ri);
for t1 = 1:ri,
    B{1,t1} = [eye(size(N{1,1},2));zeros(size(N{1,1},2))];
    for t2 = 1:ri,
        A{t1,t2} =[-N{1,t1}*(R{1,t2}')  zeros(size(N{1,1},2))
                    eye(size(N{1,1},2)) zeros(size(N{1,1},2))]; 
    end
end
nA = size(A{1,1},2);     % Determina a dimensão da matriz A
mB = size(B{1,1},2);     % Determina o número de colunas de B
% Cell que vai armazenar os controladores locais
T = cell(1,ri);
S = cell(1,ri);
% Criando as variáveis matriciais
Y = sdpvar(nA,nA,'symmetric');  % Declaracao de Y nxn simetrica
R = sdpvar(nA,nA,'full');       % Declaracao de R nxn full
% Criando as variáveis Ti, Si e Tfi
Tfi = 0;
Restr = [];
for t1 = 1:ri,
	T{1,t1} = sdpvar(nA,nA,'symmetric');  % Declaracao de Ti nxn simetrica
    S{1,t1} = sdpvar(nA,mB);              % Declaracao de Si nxm
    Tfi = Tfi + fi(1,t1)*(T{1,t1}+Y);
end
% % Criando o conjunto de restrições LMI
for t1 = 1:ri,    
    Restr = [Restr T{1,t1}>=0 T{1,t1}+Y>=0 ...
[Tfi-A{t1,t1}*R'-R*A{t1,t1}'+B{1,t1}*S{1,t1}'+S{1,t1}*B{1,t1}'  ...
(T{1,t1}-mu*(A{t1,t1}*R'-B{1,t1}*S{1,t1}')+R)';
      T{1,t1}-mu*(A{t1,t1}*R'-B{1,t1}*S{1,t1}')+R  mu*(R+R')]<=0];
    for t2 = t1+1:ri,
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