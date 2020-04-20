ControllerStruct.type    = ControlType;
ControllerStruct.sat     = Saturation;

switch ControlType
    case 'StateFeedback'
        psi=InitSTATE(8);
        K=ControlDesign_LinearStateFeedback(psi);
        
    case 'Fuzzy'
        [K,Z]=ControlDesign_Fuzzy(gamma);
        ControllerStruct.Fuzzy_Z=Z;
        ControllerStruct.Fuzzy_z={@(Psi) cos(Psi);
                                  @(Psi) sin(Psi)};
    case 'FuzzyWithZuncertainty'
        [K,Z]=ControlDesign_Fuzzy_Z_uncertainty(gamma);
        ControllerStruct.Fuzzy_Z=Z;
        ControllerStruct.Fuzzy_z={@(Psi) cos(Psi);
                                  @(Psi) sin(Psi)};
                              
    case 'FuzzywithParamUncertainty'
        [K,Z]=ControlDesign_FuzzywithParamUncertainty(gamma,MaxRotSpd);
        ControllerStruct.Fuzzy_Z=Z;
        ControllerStruct.Fuzzy_z={@(Psi) cos(Psi);
                                  @(Psi) sin(Psi)};
        
    case 'FuzzyParamUncertaintyDisturbance'
        [K,Z]=ControlDesign_FuzzyParamUncertaintyDisturbance(gamma,MaxRotSpd);
        ControllerStruct.Fuzzy_Z=Z;
        ControllerStruct.Fuzzy_z={@(Psi) cos(Psi);
                                  @(Psi) sin(Psi)};
        
    case 'LQR'
        psi=InitSTATE(8);
        K=ControlDesign_LQR(psi);
    case 'FeedbackLin'
        K=cell(2,1);
        K{1}=10*eye(4);
        K{2}=10*eye(4);
    case 'OpenLoop'
        K = zeros(4);
        ControllerStruct.Fuzzy_Z=0;
        ControllerStruct.Fuzzy_z={@(Psi) cos(Psi);
                                  @(Psi) sin(Psi)};
        disp('Controller set to open loop')
    otherwise
        K=[];
        disp('The controller you chose is not an option!')
end
ControllerStruct.Gain    = K;
% global U;

