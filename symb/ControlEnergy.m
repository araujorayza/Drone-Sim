Definitions;
n = length(N);

u=sym('u',[n 1]);
assume(u,'real');
dq = sym('dq',[n 1]);
assume(dq,'real');
q = sym('q',[n 1]);
assume(q,'real');
q(4) = psi;

f = [-N*R'*dq+M*u;
    dq];

f=simplify(f)

syms Q1 Q2 Q3 Q4 real
Q = diag([Q1 Q2 Q3 Q4])

syms P1 P2 P3 P4 real
P = diag([P1 P2 P3 P4])



V = dq'*Q*dq + q'*P*q

dV = jacobian(V,[dq;q])*f;

dV = simplify(dV)