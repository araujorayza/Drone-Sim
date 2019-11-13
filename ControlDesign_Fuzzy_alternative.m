function [K,Z] = ControlDesign_Fuzzy_alternative(gamma)
    [A,B,NRt,Z] = CalcTSModel_alternative(gamma);

    fi=0.01*ones(size(A,2));
    mu=0.1;
        in=3;

%         K = LMI_Teo51 (A,B,fi,mu,in);
    K = lmi_mozelli_alternative (A,B,fi,mu)
end