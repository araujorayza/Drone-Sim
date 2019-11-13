function dERROR = ERROR_SANTANA(t,ERROR,trajectory,Controller,gamma)
    [q_d,dq_d,ddq_d]=CalcDesTrajectory(trajectory,t);
    
    DES_POS = [dq_d;q_d];
    psi = ERROR(8)+ DES_POS(8);
    

    M = [ gamma(1)*cos(psi), -gamma(3)*sin(psi),      0,        0;
          gamma(1)*sin(psi),  gamma(3)*cos(psi),      0,        0;
                0,                  0,              gamma(5),   0;
                0,                  0,                0,    gamma(7)];

    N = [ gamma(2)*cos(psi), -gamma(4)*sin(psi),      0,        0;
          gamma(2)*sin(psi),  gamma(4)*cos(psi),      0,        0;
                0,                  0,              gamma(6),   0;
                0,                  0,                0,    gamma(8)];
            

    R = [ cos(psi),    -sin(psi),      0,    0;
          sin(psi),     cos(psi),      0,    0;
                0,          0,         1,    0;
                0,          0,         0,    1];
    
    n=length(N);
    
    A=[-N*R' zeros(n);
      eye(n) zeros(n)];
    B=[eye(n);zeros(n)];
    
    U = CalcVirtControlLaw(Controller,t,DES_POS+ERROR,ddq_d,dq_d,q_d);
   
    dERROR = A*ERROR + B*U;
end