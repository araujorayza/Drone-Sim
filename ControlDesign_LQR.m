function [K] = ControlDesign_LQR(psi)
    load gamma gamma
    N = [ gamma(2)*cos(psi), -gamma(4)*sin(psi),      0,        0;
          gamma(2)*sin(psi),  gamma(4)*cos(psi),      0,        0;
            0,                  0,              gamma(6),       0;
            0,                  0,                0,    gamma(8)];
    clear gamma
    R = [ cos(psi),    -sin(psi),      0,    0;
        sin(psi),     cos(psi),      0,    0;
        0,          0,         1,    0;
        0,          0,         0,    1];

    n=length(N);
    %dE = A*E + B*U
    A=[-N*R' zeros(n);
        eye(n) zeros(n)];
    B=[eye(n);zeros(n)];
    
    sys=ss(A,B,eye(2*n),0);
    R=10*eye(2*n);
    Q=eye(4);
    
    [K,~,~]=lqr(sys,R,Q,0);
end

