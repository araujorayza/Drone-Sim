Definitions;

NonLinQuad;

% Lets check controllability of the nonlinear quadrotor 
% model

Cont = [B A*B A^2*B A^3*B A^4*B A^5*B A^6*B A^7*B];
Cont = simplify(Cont);
Cont = rref(Cont);

if (rank(Cont) == 8)
    disp('Nonlinear QUADROTOR model: Controllability matrix is Full rank!')
end
% rank can't be trusted because it gives full rank 
% even when I repeat one matrix in the controllability

small_k = 0.1;
%Lets check controllability of the ERROR models
for ModelType=1:4
    fprintf('\nERROR MODEL #%d',ModelType)
    [A,B,h] = ErrorModeling(ModelType,gamma);
    for i = 1:size(A,1)
        for j = 1:size(A,2)
            Cont = [B{j} A{i,j}*B{j} A{i,j}^2*B{j} ...
                A{i,j}^3*B{j} A{i,j}^4*B{j} A{i,j}^5*B{j}...
                A{i,j}^6*B{j} A{i,j}^7*B{j}];
            Cont = simplify(Cont);
            Cont = rref(Cont);
            if(rank(Cont) == 8)
                fprintf('\n\tlocal model (%d,%d): Controllability matrix is Full rank!\n',i,j)
            else
                fprintf('\n\t--> NON CONTROLLABLE! local model (%d,%d): Controllability matrix rank is %d\n',i,j,rank(Cont))
            end
            
        end
    end
    
    
end

