Definitions;

ModelType = 4
small_k = 1;

fprintf('Nonlinear error model dx = Ax + Bu\n with x = [de e]^T\n');

n=length(N);

Anl=[-N*R' -small_k*eye(n);
  eye(n) zeros(n)];

Bnl=[M;zeros(n)];

%assumeAlso(gamma(2) > gamma(4))
[A,B,h] = ErrorModeling(ModelType,gamma);
[Ah,Bh] = Defuzzy(A,B,h,ModelType);

simplify(Anl - Ah)



