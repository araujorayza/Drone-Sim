function dE = model_nonlinear(t,E)
    NonlinearMatrices;
    [q_d,dq_d,~]=CalcDesTrajectory('circle',t);
    DES_STATE = [dq_d;q_d];
    psi = E(8) + DES_STATE(8);
    
    A = [-N(psi)*R(psi)' zeros(4);
                eye(4)   zeros(4)];
    B = [eye(4);zeros(4)];
    
    U=ones(4,1);
    
    dE=A*E+ B*U;
end

