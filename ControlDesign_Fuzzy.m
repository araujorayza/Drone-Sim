function [K,Z] = ControlDesign_Fuzzy(gamma)
    [A,B,M,N,R,Z] = CalcTSModel(gamma);

    fi=0.01*ones(size(A,2));
    mu=0.1;
%     in=3;

%     K = LMI_Teo51 (A,B,fi,mu,in);
    K = lmi_mozelli (M,N,R,fi,mu);
end