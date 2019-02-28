function [K,Z] = ControlDesign_Fuzzy(gamma)
    [A,B,Z] = CalcTSModel(gamma);

    fi=10*ones(size(A,2));
    mu=10;
    in=3;

    K = LMI_Teo51 (A,B,fi,mu,in);
end