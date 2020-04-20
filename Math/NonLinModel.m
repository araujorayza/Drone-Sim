Definitions;

disp('O modelo eh dado por dx = Ax + Bv, com x = [dq q]^T');

n=length(N);

A=simplify(...
  [-N*R' zeros(n);
  eye(n) zeros(n)]);
pretty(A)
 
B=[M;zeros(n)]




