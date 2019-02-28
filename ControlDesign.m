ControllerStruct.type    = ControlType;
switch ControlType
    case 'StateFeedback'
        psi=InitSTATE(8);
        K=ControlDesign_LinearStateFeedback(psi);
        
    case 'Fuzzy'
        [K,Z]=ControlDesign_Fuzzy(gamma);
        ControllerStruct.Fuzzy_Z=Z;
        ControllerStruct.Fuzzy_z={@(Psi) gamma(2)*cos(Psi)^2+gamma(4)*sin(Psi)^2;
                                  @(Psi) (gamma(2)-gamma(4))*sin(2*Psi)/2};
                              
    case 'OpenLoop'
        K = zeros(4);
    otherwise
        K=[];
        disp('The controller you chose is not an option!')
end
ControllerStruct.Gain    = K;

