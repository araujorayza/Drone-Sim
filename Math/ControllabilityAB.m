Definitions;

NonLinModel;

%Lets check controllability in (A,B)

Cont = [B A*B A^2*B A^3*B A^4*B A^5*B A^6*B A^7*B];
Cont = simplify(Cont);
Cont = rref(Cont);

rank(Cont)

% rank can't be trusted because it gives full rank 
% even when I repeat one matrix in the controllability