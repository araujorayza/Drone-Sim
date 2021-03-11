Definitions;

fprintf('Nonlinear quadrotor model dx = Ax + Bu\n with x = [dq q]^T\n');

n=length(N);

A=[-N*R' zeros(n);
  eye(n) zeros(n)];

B=[M;zeros(n)];

C=[zeros(n) eye(n)];





