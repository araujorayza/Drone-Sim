function dx = nonmin_phase(t,x)
    rho = cos(2*t);
    u = 1 + exp(t)*sin(t);
    
    A = [0, (2-rho)^2, 1 + 0.5*rho + (2-rho)^2;
     1,         0,              0.2;
     0,0,0];
    B = [0 0 1]';
    C = [0 1 1];

    dx = A*x+B*u;
end

