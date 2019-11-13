function dERROR = ERROR_Fuzzy(t,ERROR,trajectory,Controller,gamma)
    [~,B,M,N,R,Z] = CalcTSModel(gamma);
    
    [q_d,dq_d,ddq_d]=CalcDesTrajectory(trajectory,t);
    DES_POS = [dq_d;q_d];
    psi = ERROR(8)+ DES_POS(8);
    
    min=1;max=2;
%     Z = Controller.Fuzzy_Z;
    z = {@(Psi) cos(Psi);
         @(Psi) sin(Psi)};
        
    H = zeros(length(z),2);
    for i=1:length(z)
        H(i,min) = (z{i}(psi)-Z(i,min))/(Z(i,max)-Z(i,min)); %this is inverted like this. 
        H(i,max) = 1 - H(i,1);
    end
    
    h=CalcDefuzzWeights(H);
    
    K=Controller.Gain;
       
    STATE = ERROR + DES_POS;
    U = CalcVirtControlLaw(Controller,t,STATE,ddq_d,dq_d,q_d);
    
    dERROR = 0*ERROR;
    Nh     = 0*N{1,1};
    Rh     = 0*R{1,1};
    Bh     = 0*B{1,1};
    
    for k=1:length(h)
        Nh=Nh+h(k)*N{1,k};
        Rh=Rh+h(k)*R{1,k};
        Bh=Bh+h(k)*B{1,k};
    end
    dERROR = [-Nh*Rh' zeros(4);eye(4) zeros(4)]*ERROR + Bh*U;
end