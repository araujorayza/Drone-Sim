function [dSTATE,U] = DRONE_SANTANA(t,STATE,Simu)
    global small_k;
    gamma = Simu.sys.param;
    psi=STATE(8);

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
    B=[M;zeros(n)];
    

    [q_d,dq_d,ddq_d]=CalcDesTrajectory(Simu.trajectory,t);
    V = CalcVirtControlLaw(Simu.controller,t,STATE,[dq_d;q_d]);
    U=M\(V + N*R'*dq_d + ddq_d - small_k*(STATE(5:8)-q_d));
    
     
    dSTATE = A*STATE + B*U;
end