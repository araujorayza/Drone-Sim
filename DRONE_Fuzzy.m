function dSTATE = DRONE_Fuzzy(t,STATE,trajectory,Controller,gamma) %this function is not working and is not called anywhere
    [A,B,M,N,R,Z] = CalcTSModel(gamma);
    
%     [q_d,dq_d,ddq_d]=CalcDesTrajectory(trajectory,t);
   
    U = CalcVirtControlLaw(Controller,t,STATE,ddq_d,dq_d,q_d);
    
%     V=M\(U + N*R'*dq_d + ddq_d);
    
    min=1;max=2;
    
    Z = Controller.Fuzzy_Z;
    z = Controller.Fuzzy_z;
    
    psi = STATE(8);
    
    H = zeros(length(z),2);
    for i=1:length(z)
        H(i,min) = (z{i}(psi)-Z(i,min))/(Z(i,max)-Z(i,min)); %this is inverted like this. 
        H(i,max) = 1 - H(i,1);
    end
    
    h=CalcDefuzzWeights(H);

    dSTATE = 0*STATE;
    for i=1:4
        dSTATE = dSTATE + h(i)*(A{}*STATE+);
    end
    
end