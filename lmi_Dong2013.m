function L = lmi_Dong2013 (A,B,C,mu)
rp = size(A,2);
% Condi��es de estabilidade baseadas no Dong 2013 com 
nA = size(A{1,1},2);
nB = size(B{1,1},2);
nC = size(C,1);

% Criando as vari�veis matriciais
  G = sdpvar(nC,nC,'full');
  T = inv(C*C'); % Matriz definida em (9) de [J. Dong and G. Yang, 2013]
  Q = cell(1,rp);
  L = sdpvar(nB,nC,'full');
Restr = [];
for t1=1:rp
    Q{1,t1} = sdpvar(nA,nA,'symmetric');
    Restr = [Restr,Q{1,t1}>=0];
end
% Criando as estruturas matriciais usadas nas LMIs
LMI = cell(rp,rp);
for t1=1:rp
    for t2=1:rp
      a11=A{1,t1}*Q{1,t2}+Q{1,t2}*A{1,t1}'+...
          B{1,t1}*L*T*C+(B{1,t1}*L*T*C)';
      a21=C*Q{1,t1}-G*T*C+mu*(B{1,t1}*L)';
      a22=-mu*(G+G');
      LMI{t1,t2} = [a11 a21';a21 a22];
    end
end
% Criando as LMIs
for t1=1:rp
    for t2=t1:rp
        Restr = [Restr,LMI{t1,t2}+LMI{t2,t1}<=0];
    end
end
% Configurando o Solver.
opts=sdpsettings;
opts.savesolverinput=1;
opts.savesolveroutput=1;
% opts.solver='lmilab';
% opts.solver='sedumi';
opts.verbose=0;
% Resolvendo as LMIs
sol = optimize(Restr,[],opts);
che=min(check(Restr));
if che > 0
      disp('DONG2013 - SIM')
    % Encontra o valor num�rico das matrizes
    G = value(G);
    L = value(L)*inv(G);
    for i1=1:rp,
       Q{1,i1}  = value(Q{1,i1});
    end
else
    L=[];
    disp('DONG2013 - N�O')
end