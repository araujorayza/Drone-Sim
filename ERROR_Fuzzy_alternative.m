function dERROR = ERROR_Fuzzy_alternative(t,ERROR,trajectory,Controller,gamma)
    [A,B,NRt,Z] = CalcTSModel_alternative(gamma);
    
    [q_d,dq_d,ddq_d]=CalcDesTrajectory(trajectory,t);
    DES_POS = [dq_d;q_d];
    psi = ERROR(8)+ DES_POS(8);
    
    min=1;max=2;
%     z = {@(Psi) gamma(2)*cos(Psi)^2+gamma(4)*sin(Psi)^2;
%          @(Psi) (gamma(2)-gamma(4))*sin(2*Psi)/2};
    
    z = Controller.Fuzzy_z;
    
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
    Ah     = 0*A{1,1};
    Bh     = 0*B{1,1};
    
    for k=1:length(h)
        Ah = Ah + h(k)*A{1,k};
        Bh = Bh + h(k)*B{1,k};
    end
    dERROR = Ah*ERROR + Bh*U;
end