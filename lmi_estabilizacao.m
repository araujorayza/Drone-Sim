function [Q,W,K] = lmi_estabilizacao (N,R,mu,fi,V)
    %LMIs implemeted based on Theorem [[]] from paper "------" ICUAS 2020
% As LMIs s�o formuladas para o modelo do erro de rastreamento
[LN CN]=size(N{1,1});
    ri = size(N,2); % N�mero de modelos locais
     p = log2(ri);  % N�mero de n�o linearidades no modelo original
    nv = size(V,2); % N�mero de colunas da matriz V
% Criandos as matrizes aumentadas do modelo do erro
     A = cell(2,ri,ri);
    B2 = cell(1,ri);
for t1 = 1:ri,
    B2{1,t1} = [eye(LN);zeros(CN)];
    for t2 = 1:ri,
        % Incerteza 1
        A{1,t1,t2} = [-N{1,t1}*(R{1,t2}')  zeros(LN,CN)
                            eye(LN)        zeros(LN,CN)];
        % Incerteza 2
        A{2,t1,t2} = [-N{2,t1}*(R{1,t2}')  zeros(LN,CN)
                            eye(LN)        zeros(LN,CN)];
    end
end
% L* = N. de linhas da matriz *
% C* = N. de colunas da matriz *
LA  = size(A{1,1,1},1);
CB2 = size(B2{1,1},2);
% Criando as vari�veis matriciais
LMI = cell(2,nv,ri,ri);
  Q = cell(1,ri);
  Y = cell(1,ri);
W = sdpvar(LA,LA,'full');
Restr =[];
for i1=1:ri,
    Q{1,i1}= sdpvar(LA,LA,'symmetric');
    Y{1,i1}= sdpvar(CB2,LA,'full');
    Restr = [Restr Q{1,i1}>=0];
end
for t1=1:nv
  for ik=1:ri,
    for ii=1:ri,
        % Variaveis matriciais para a incerteza 1
        a11=A{1,ik,ii}*W+W'*A{1,ik,ii}'-B2{1,ik}*Y{1,ii}-Y{1,ii}'*B2{1,ik}';
        a21=Q{1,ik}-W'+mu*((A{1,ik,ii}*W-B2{1,ik}*Y{1,ii}));
        a22=-mu*(W+W');
        LMI{1,t1,ik,ii}=[a11 a21'
                         a21 a22];
        % Variaveis matriciais para a incerteza 2
        a11=A{2,ik,ii}*W+W'*A{2,ik,ii}'-B2{1,ik}*Y{1,ii}-Y{1,ii}'*B2{1,ik}';
        a21=Q{1,ik}-W'+mu*((A{2,ik,ii}*W-B2{1,ik}*Y{1,ii}));
        a22=-mu*(W+W');
        LMI{2,t1,ik,ii}=[a11 a21'
                         a21 a22];
        for t2=1:ri
            % Incerteza 1
            LMI{1,t1,ik,ii}=LMI{1,t1,ik,ii}+...
                          [(V(t2,t1))*fi(1,1)*Q{1,t2} zeros(LA,LA);
                                         zeros(LA,LA) zeros(LA,LA)];
            % Incerteza 2
            LMI{2,t1,ik,ii}=LMI{2,t1,ik,ii}+...
                          [(V(t2,t1))*fi(1,2)*Q{1,t2} zeros(LA,LA);
                                         zeros(LA,LA) zeros(LA,LA)];
        end
        
    end
  end
end
% Criando as LMIs
for t1=1:nv
  for ik=1:ri,
    for ii=1:ri,
        if ik==ii
            % Incerteza 1
            Restr=[Restr LMI{1,t1,ik,ik}<=0];
            % Incerteza 2
            Restr=[Restr LMI{2,t1,ik,ik}<=0];
        else
            % Incerteza 1
            Restr=[Restr (2/(2^p-1))*LMI{1,t1,ik,ik}+LMI{1,t1,ik,ii}+...
                                                     LMI{1,t1,ii,ik}<=0];
            % Incerteza 2
            Restr=[Restr (2/(2^p-1))*LMI{2,t1,ik,ik}+LMI{2,t1,ik,ii}+...
                                                     LMI{2,t1,ii,ik}<=0];
        end
    end
  end
end
% Configurando o Solver.
opts=sdpsettings;
% opts.solver='lmilab';
opts.verbose=0;
% Resolvendo as LMIs
sol = solvesdp(Restr,[],opts);
che=min(checkset(Restr));
if che > 0
    % Encontra o valor num�rico das matrizes
    W=double(W);
    K = cell(1,ri);
    for i1=1:ri,
        Q{1,i1} = double(Q{1,i1});
        K{1,i1} = double(Y{1,i1})*inv(W);
    end
else
    display('LMIs infact�veis')
    Q=[]; W=[]; K=[];
end