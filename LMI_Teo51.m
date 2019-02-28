function K = LMI_Teo51 (A,B,fi,mu,in)
% TEOREMA 5.1 DO NOSSO ARTIGO
% Esta função encontra condições suficientes para o projeto de
% controladores fuzzy, considerando realimentação de estados.
% Esse trabalho implementa uma "relaxação" do resultado do Teorema 6 de:
% A systematic approach to improve multiple Lyapunov function stability
% and stabilization conditions for fuzzy systems
% L. A. Mozelli, R. M. Palhares, G. S. C. Avellar. 2009
% -------------------------------------------------------------------

ri = size(A,2);       % Determina o número de modelos locais do sistema
nA = size(A{1,1},2);  % Determina a dimensão da matriz A
mB = size(B{1,1},2);  % Determina o número de colunas de B
% Cell que vai armazenar os controladores locais
T = cell(1,ri);
S = cell(1,ri);
% Criando as variáveis matriciais
Y = sdpvar(nA,nA,'symmetric');      % Declaracao de Y nxn simetrica
R = sdpvar(nA,nA,'full');      % Declaracao de R nxn full
% Criando as variáveis Ti, Si e Tfi
Tfi1 =  fi(in,1)*Y;
Tfi2 = -fi(in,1)*Y;
% Criando as variáveis definidas positivas Ti e as variáveis Si
Restr = [];
for t1 = 1:ri,
	T{1,t1} = sdpvar(nA,nA,'symmetric');  % Declaracao de Ti nxn simetrica
    S{1,t1} = sdpvar(nA,mB);              % Declaracao de Si nxm
    Restr = [Restr T{1,t1}>=0];
end
% Criando a variável Tfi
for t1 = 1:ri,
    if t1~=in
       Tfi1 = Tfi1 + fi(t1,1)*(T{1,t1}-T{1,in}+Y);
       Tfi2 = Tfi2 + fi(t1,1)*(T{1,t1}-T{1,in}+Y);
       Restr = [Restr T{1,t1}-T{1,in}+Y>=0];
    end
end
% % Criando o conjunto de restrições LMI
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
    disp('O sistema não pode ser controlado')
end