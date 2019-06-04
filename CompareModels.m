NonlinearModel

%variar psi
%variar vetor E
psi = linspace(0,2*pi,20);

nu = ones(4,1);
E  = ones(8,1);

dE = @(E,psi) [-N(psi)*R(psi)', zeros(4);
                eye(4),        zeros(4)]*E + [eye(4);zeros(4)]*nu;


for i=1:length(psi)
    E_ponto(:,i) = dE(E,psi(i));
end


TSModel
dE_fuzzy = zeros(8,length(psi));
h=zeros(4,length(psi));

for k=1:length(psi)
    for i=1:length(z)
        H(i,1) = (z{i}(psi(k))-Z(i,min))/(Z(i,max)-Z(i,min));
        H(i,2) = 1 - H(i,1);
    end
    h(:,k)=CalcDefuzzWeights(H);
end 

for k=1:length(psi)
    for i=1:ri
        dE_fuzzy(:,k) = dE_fuzzy(:,k) + h(i,k)*A{1,i}*E+B{1,i}*nu;
    end
end

plot(E_ponto(1,:),psi,...
    dE_fuzzy(1,:),psi)
