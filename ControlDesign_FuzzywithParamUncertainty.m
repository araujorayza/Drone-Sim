function [K,Z] = ControlDesign_FuzzywithParamUncertainty(gamma, MaxRotSpd)
    %LMIs implemeted based on Theorem [[]] from paper "------" ICUAS 2020
    
    % Limits for the varying parameters
    gammaMax = 2*gamma;
    gammaMin = -2*gamma;
    
    
    % Calculating the model for max and min parameters
    [A1,B1,M1,N1,R1,Z1] = CalcTSModel(gammaMin);
    [A2,B2,M2,N2,R2,Z2] = CalcTSModel(gammaMax);

    
    % LMI design 
    p   = size(Z1,1);            % number of nonlinearities
    fi  = 0.95*ones(size(A1,2)); % max value of derivative of membership functions (calculated graphically)
    mu  = 1;                   % 
    
    
    N = { N1{:,:}; N2{:,:} };
    R = { R1{:,:}; R2{:,:} };
    
    
    V=function_total_matrix(p);
    
    [Q,W,K] = lmi_estabilizacao2(N,R,mu,fi,V);
    Z=Z1; %cause it's the same 
end