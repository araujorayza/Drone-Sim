NonLinQuad;
C=[zeros(n) eye(n)];

zeta = sym('zeta',[2*n 1]);
zeta(8) = psi;

u = sym('u',[n 1]);

f=A*zeta+B*u;


for i=1:length(zeta)
    DA(:,i) = diff(f,zeta(i));
end
DA = simplify(DA)

for i=1:length(u)
    DB(:,i) = diff(f,u(i));
end
DB = simplify(DB)

zeta_eq = sym('zeta',[2*n 1]);
zeta_eq = [zeros(n,1);zeta_eq(n+1:2*n)];
u_eq = 0*u;

DA=subs(DA,zeta,zeta_eq);
DA=subs(DA,u,u_eq)
latex(DA)


DB=subs(DB,zeta,zeta_eq);
DB=subs(DB,u,u_eq)
latex(DB)


disp('eigenvalues of linearized A:')
eigen_DA = eig(DA);
eigen_DA = simplify(eigen_DA)


disp('controllability')

Cont = [DB DA*DB DA^2*DB DA^3*DB DA^4*DB DA^5*DB DA^6*DB DA^7*DB];
Cont = simplify(Cont)
rank(Cont)
latex(Cont')
Cont = rref(Cont);
Cont = simplify(Cont);
rank(Cont)

disp('output controllability')

Cont = C*[DB DA*DB DA^2*DB DA^3*DB DA^4*DB DA^5*DB DA^6*DB DA^7*DB];
Cont = simplify(Cont)
rank(Cont)
latex(Cont)
Cont = rref(Cont);
Cont = simplify(Cont);
rank(Cont)