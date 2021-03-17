Definitions;

NonLinQuad;

u=sym('u',[n 1]);
zeta = sym('zeta',[2*n 1]);
zeta(8) = psi;


resp = solve(B,psi)

resp = solve(B*u,psi)

f = A*zeta
g = B*u


function r=liebr(f,g,x)
    r = jacobian(g,x)*f-diff(f,x)*g
end