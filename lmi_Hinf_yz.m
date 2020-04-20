function [Q,W,K] = lmi_Hinf_yz (N,R,C,mu,fitilde,Gtilde,ga)
% TESTANDO UMAS LMIS OBTIDAS COM O M�TODO DE DESACOPLAMENTO (Mozelli, 2009)
    [LN CN]=size(N{1,1});
    ri = size(N,2); % N�mero de modelos locais
    p = log2(ri);  % N�mero de n�o linearidades no modelo original
    nv = size(Gtilde,2); % N�mero de colunas da matriz V
    ri = size(N,2);
    A = cell(2,ri,ri);
    B1 = cell(2,ri,ri);
    Bw = cell(1,ri);
    
    B   = [eye(LN);zeros(CN)];
    Bw  = [zeros(LN);eye(CN)]; %The wind disturbance changes velocity%
        
    for t1 = 1:ri,
        for t2 = 1:ri,
            % Incerteza 1
            A{1,t1,t2} = [-N{1,t1}*(R{1,t2}')  zeros(LN,CN)
                              eye(LN)          zeros(LN,CN)];
            
            % Incerteza 2
            A{2,t1,t2} = [-N{2,t1}*(R{1,t2}')  zeros(LN,CN)
                              eye(LN)          zeros(LN,CN)];
        end
    end
    % L* = N. de linhas da matriz *
    % C* = N. de colunas da matriz *
    LA  = size(A{1,1,1},2);
    CB1 = size(B1{1,1,1},2);
    CBw = size(Bw{1,1},2);
    LC  = size(C,1);

    % Criando as vari�veis matriciais
    Psi = cell(2,nv,ri,ri);

    Q = cell(1,ri);
    Y = cell(1,ri);
    W = sdpvar(LA,LA,'full');
    Restr =[];
    for i=1:ri,
        Q{1,i}= sdpvar(LA,LA,'symmetric');
        Y{1,i}= sdpvar(CBw,LA,'full');
        Restr = [Restr Q{1,i}>=0];
    end

    Qtilde = cell(1,2);
    for l=1:nv
        Qtilde{1,l} = 0*fitilde*Gtilde(i,l)*Q{1,i}; %inicialization
        for i=1:ri
            Qtilde{1,l} = Qtilde{1,l} + fitilde*Gtilde(i,l)*Q{1,i};
        end
    end
    
    
    for k=1:2
        for l=1:nv
          for i=1:ri
            for j=1:ri
                
                a11=A{k,ik,ii}*W+W'*A{k,ik,ii}'-Bw{1,ik}*Y{k,ii}-Y{k,ii}'*Bw{1,ik}';
                a21=Q{1,ik}-W'+mu*((A{k,ik,ii}*W-Bw{k,ik}*Y{1,ii}));
                a22=-mu*(W+W');
                a31=B1{k,ik,ii}';
                a32=mu*a31;
                a33=-eye(CB1)/(ga^2);
                a41=C*W;
                a42=zeros(LC,LA);
                a43=zeros(LC,CB1);
                a44=-eye(LC);
                Psi{k,l,i,j}=[a11 a21' a31' a41'
                                 a21 a22  a32' a42'
                                 a31 a32  a33  a43'
                                 a41 a42  a43  a44];

                % Variaveis matriciais para a incerteza 2
                a11=A{2,ik,ii}*W+W'*A{2,ik,ii}'-Bw{1,ik}*Y{1,ii}-Y{1,ii}'*Bw{1,ik}';
                a21=Q{1,ik}-W'+mu*(A{2,ik,ii}*W-Bw{1,ik}*Y{1,ii});
                a22=-mu*(W+W');
                a31=B1{2,ik,ii}';
                a32=mu*a31;
                a33=-eye(CB1)/(ga^2);
                a41=C*W;
                a42=zeros(LC,LA);
                a43=zeros(LC,CB1);
                a44=-eye(LC);
                Psi{2,t1,ik,ii}=[a11 a21' a31' a41'
                                 a21 a22  a32' a42'
                                 a31 a32  a33  a43'
                                 a41 a42  a43  a44];
                for t2=1:ri
                  Qfi=[Q{1,t2}        zeros(LA,LA)  zeros(LA,CB1)  zeros(LA,LC)  
                       zeros(LA,LA)   zeros(LA,LA)  zeros(LA,CB1)  zeros(LA,LC)
                       zeros(CB1,LA)  zeros(CB1,LA) zeros(CB1,CB1) zeros(CB1,LC)
                       zeros(LC,LA)   zeros(LC,LA)  zeros(LC,CB1)  zeros(LC,LC)];
                    % Incerteza 1
                    Psi{1,t1,ik,ii}=Psi{1,t1,ik,ii}+Gtilde(t2,t1)*fitilde(1,1)*Qfi;
                    % Incerteza 2
                    Psi{2,t1,ik,ii}=Psi{2,t1,ik,ii}+Gtilde(t2,t1)*fitilde(1,2)*Qfi;
                end
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
                Restr=[Restr Psi{1,t1,ik,ik}<=0];
                % Incerteza 2
                Restr=[Restr Psi{2,t1,ik,ik}<=0];
            else
                % Incerteza 1
                Restr=[Restr (2/(2^p-1))*Psi{1,t1,ik,ik}+Psi{1,t1,ik,ii}+...
                                                         Psi{1,t1,ii,ik}<=0];
                % Incerteza 2
                Restr=[Restr (2/(2^p-1))*Psi{2,t1,ik,ik}+Psi{2,t1,ik,ii}+...
                                                         Psi{2,t1,ii,ik}<=0];
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
        for i=1:ri,
            Q{1,i} = double(Q{1,i});
            K{1,i} = double(Y{1,i})*inv(W);
        end
    else
        display('LMIs infact�veis')
        Q=[]; W=[]; K=[];
    end