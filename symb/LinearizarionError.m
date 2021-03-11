NonLinQuad;

fprintf('Nonlinear ERROR model dx = Ax + Bu\n with x = [de e]^T\n');
clear small_k
syms small_k real


A=[-N*R' -small_k*eye(n);
  eye(n) zeros(n)];

B=[eye(n);zeros(n)];

C=[zeros(n) eye(n)];

e = sym('x',[2*n 1]);
e(8) = psi;

nu = sym('nu',[n 1]);

f=A*e+B*nu;


for i=1:length(e)
    DA(:,i) = diff(f,e(i));
end
DA = simplify(DA);

for i=1:length(nu)
    DB(:,i) = diff(f,nu(i));
end

DA=subs(DA,e,0*e);
DA=subs(DA,nu,0*nu);

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