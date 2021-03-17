Definitions;

ModelType = 2
small_k = 1;

fprintf('Nonlinear error model dx = Ax + Bu\n with x = [de e]^T\n');

n=length(N);

%assumeAlso(gamma(2) > gamma(4))
[A,B,h] = ErrorModeling(ModelType,gamma);
[Ah,Bh] = Defuzzy(A,B,h,ModelType);

celldisp(A)

