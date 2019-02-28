function K = ControlDesign_LinearStateFeedback(psi)

    load gamma gamma
    N = [ gamma(2)*cos(psi), -gamma(4)*sin(psi),      0,        0;
        gamma(2)*sin(psi),  gamma(4)*cos(psi),      0,        0;
        0,                  0,              gamma(6),   0;
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

    P=-1:-1:-8;
    K=place(A,B,P);
end 