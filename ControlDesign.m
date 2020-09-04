if(Modeltype == 1)
    [A,B,h] = CalcTSModel(gamma);
elseif(Modeltype == 2)
    
end

switch ControlType
    case 'StateFeedback'
        psi=InitSTATE(8);
        K=ControlDesign_LinearStateFeedback(psi);
        
    case 'Fuzzy'
        fi=0.01*ones(size(A,2));
        mu=0.1;
        in=3;

        K = LMI_Teo51 (A,B,fi,mu,in);  
        
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
        
    case 'openloop'
        K = zeros(4);
        disp('Controller set to open loop')
        
    otherwise
        K=[];
        disp('The controller you chose is not an option!')
end
SimStruct.sys.sat = Saturation;

SimStruct.controller.type = ControlType;
SimStruct.controller.model.type = Modeltype;
SimStruct.controller.model.A = A;
SimStruct.controller.model.B = B;
SimStruct.controller.model.h = h;
SimStruct.controller.gain    = K;