function dSTATE = DRONE_SANTANA(t,STATE,Simu)%trajectory,Controller,gamma,WindDisturbance)
    gamma = Simu.sys.param;
    trajectory = Simu.trajectory;
    controller = Simu.controller;
    

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
    

    U = CalcVirtControlLaw(controller,t,STATE,ddq_d,dq_d,q_d);
    
    
    V=M\(U + N*R'*dq_d + ddq_d);
    
    if (controller.sat)
        V=saturate_control(V);
    end
    
%     hold on
%     plot(t,V,'dk') %plot control law
     
    dSTATE = A*STATE + B*V;
    
    
    if(Simu.WindDisturbance.Type)
        dSTATE(5:8) = dSTATE(5:8) + WindDisturbance(t,Simu.WindDisturbance);
    end

end