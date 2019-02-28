function dSTATE = DRONE_Fuzzy(t,STATE,trajectory,Controller,gamma)
    [A,B,Z] = CalcTSModel(gamma);
    
    [q_d,dq_d,ddq_d]=CalcDesTrajectory(trajectory,t);
   
    U = CalcVirtControlLaw(Controller,t,STATE,[dq_d;q_d]);
    
    V=M\(U + N*R'*dq_d + ddq_d);
    Af
    Bf
    dSTATE = A*STATE + B*V;
end