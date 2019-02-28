
M = @(psi)[ gamma(1)*cos(psi), -gamma(3)*sin(psi),      0,        0;
            gamma(1)*sin(psi),  gamma(3)*cos(psi),      0,        0;
                0,                  0,              gamma(5),   0;
                0,                  0,                0,    gamma(7)];

N = @(psi)[ gamma(2)*cos(psi), -gamma(4)*sin(psi),      0,        0;
          gamma(2)*sin(psi),  gamma(4)*cos(psi),      0,        0;
                0,                  0,              gamma(6),   0;
                0,                  0,                0,    gamma(8)];
            
R = @(psi)[ cos(psi),    -sin(psi),      0,    0;
            sin(psi),     cos(psi),      0,    0;
                  0,          0,         1,    0;
                  0,          0,         0,    1];

{@(Psi) gamma(2)*cos(Psi)^2+gamma(4)*sin(Psi)^2;
                                  @(Psi) (gamma(2)-gamma(4))*sin(2*Psi)/2};