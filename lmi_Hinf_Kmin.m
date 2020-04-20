function [Q,W,K] = lmi_Hinf_Kmin (N,R,C,mu,fi,V,ga,beta1,beta2)
% TESTANDO UMAS LMIS OBTIDAS COM O M�TODO DE DESACOPLAMENTO (Mozelli, 2009)
[LN,CN]=size(N{1,1});
    ri = size(N,2); % N�mero de modelos locais
     p = log2(ri);  % N�mero de n�o linearidades no modelo original
    nv = size(V,2); % N�mero de colunas da matriz V
    ri = size(N,2);
     A = cell(2,ri,ri);
    B1 = 0.1*[zeros(LN);eye(CN)]; %The wind disturbance changes velocity
    B2 = [eye(LN);zeros(CN)];
for t1 = 1:ri,
    for t2 = 1:ri,
        % Incerteza 1
        A{1,t1,t2} = [-N{1,t1}*(R{1,t2}')  zeros(LN,CN)
                          eye(LN)          zeros(LN,CN)];
%         B1{1,t1,t2} = [-eye(LN)       -N{1,t1}*(R{1,t2}')
%                        zeros(LN,CN)   zeros(LN,CN)];
        % Incerteza 2
        A{2,t1,t2} = [-N{2,t1}*(R{1,t2}')  zeros(LN,CN)
                          eye(LN)          zeros(LN,CN)];
%         B1{2,t1,t2} = [-eye(LN)       -N{2,t1}*(R{1,t2}')
%                        zeros(LN,CN)   zeros(LN,CN)];
    end
end
% L* = N. de linhas da matriz *
% C* = N. de colunas da matriz *
LA  = size(A{1,1,1},2);
CB1 = size(B1,2);
CB2 = size(B2,2);
LC  = size(C,1);

% Criando as vari�veis matriciais
LMI = cell(2,nv,ri,ri);
  Q = cell(1,ri);
  Y = cell(1,ri);
W = sdpvar(LA,LA,'full');
Restr = [];
for i1=1:ri,
    Q{1,i1}= sdpvar(LA,LA,'symmetric');
    Y{1,i1}= sdpvar(CB2,LA,'full');
    % Restri��es que minimizam a norma do ganho do controlador
    Restr = [Restr Q{1,i1}>=beta1*eye(LA)];
    Restr = [Restr [beta2*eye(CB2) Y{1,i1};Y{1,i1}' eye(LA)]>=0];
end
for t1=1:nv
  for ik=1:ri
    for ii=1:ri
        % Variaveis matriciais para a incerteza 1
        a11=-A{1,ik,ii}*W-W'*A{1,ik,ii}'+B2*Y{1,ik}+Y{1,ik}'*B2';
        a21=Q{1,ik}+W'-mu*(A{1,ik,ii}*W-B2*Y{1,ik});
        a22=mu*(W+W');
        a31=-B1';
        a32=mu*a31;
        a33=-eye(CB1)*ga^2;
        a41=C*W;
        a42=zeros(LC,LA);
        a43=zeros(LC,CB1);
        a44=-eye(LC);
        LMI{1,t1,ik,ii}=[a11 a21' a31' a41'
                         a21 a22  a32' a42'
                         a31 a32  a33  a43'
                         a41 a42  a43  a44];

        % Variaveis matriciais para a incerteza 2
        a11=-A{2,ik,ii}*W-W'*A{2,ik,ii}'+B2*Y{1,ik}+Y{1,ik}'*B2';
        a21=Q{1,ik}+W'-mu*(A{2,ik,ii}*W-B2*Y{1,ik});
        a22=mu*(W+W');
        a31=-B1';
        a32=mu*a31;
        a33=-eye(CB1)*ga^2;
        a41=C*W;
        a42=zeros(LC,LA);
        a43=zeros(LC,CB1);
        a44=-eye(LC);
        LMI{2,t1,ik,ii}=[a11 a21' a31' a41'
                         a21 a22  a32' a42'
                         a31 a32  a33  a43'
                         a41 a42  a43  a44];
        for t2=1:ri
          Qfi=[Q{1,t2}        zeros(LA,LA)  zeros(LA,CB1)  zeros(LA,LC)  
               zeros(LA,LA)   zeros(LA,LA)  zeros(LA,CB1)  zeros(LA,LC)
               zeros(CB1,LA)  zeros(CB1,LA) zeros(CB1,CB1) zeros(CB1,LC)
               zeros(LC,LA)   zeros(LC,LA)  zeros(LC,CB1)  zeros(LC,LC)];
            % Incerteza 1
            LMI{1,t1,ik,ii}=LMI{1,t1,ik,ii}+V(t2,t1)*fi(1,1)*Qfi;
            % Incerteza 2
            LMI{2,t1,ik,ii}=LMI{2,t1,ik,ii}+V(t2,t1)*fi(1,2)*Qfi;
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