function dSTATE = DRONE_SANTANA(t,STATE,trajectory,Controller,gamma)
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
    
    [q_d,dq_d,ddq_d]=CalcDesTrajectory(trajectory,t);
%     STATE=KalmanFilter(STATE);
    U = CalcVirtControlLaw(Controller,t,STATE,ddq_d,dq_d,q_d);
    
    V=M\(U + N*R'*dq_d + ddq_d);
    
    if (Controller.sat)
        V=saturate_control(V);
    end
        
    plot(t,V,'dk')
    dSTATE = A*STATE + B*V;
end