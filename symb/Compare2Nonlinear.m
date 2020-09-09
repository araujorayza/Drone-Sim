Definitions;
ModelType = 2


fprintf('Nonlinear quadrotor model dx = Ax + Bu\n with x = [dq q]^T\n');

n=length(N);

Anl=[-N*R' zeros(n);
  eye(n) zeros(n)];

Bnl=[M;zeros(n)];

%assumeAlso(gamma(2) > gamma(4))
[A,B,h] = ErrorModeling(ModelType,gamma);
[Ah,Bh] = Defuzzy(A,B,h,ModelType);

simplify(Anl - Ah)



