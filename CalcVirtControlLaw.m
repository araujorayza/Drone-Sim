function U = CalcVirtControlLaw(Controller,t,State,DesState)
    K=Controller.gain;
    switch Controller.type
        case 'openloop'
            U=ones(length(t)).*[sin(t)+sin(1.3*t);
                -cos(4*t)-cos(5.3*t);
                sin(2*t)+sin(2.3*t);
                cos(0.5*t)+cos(0.3*t)];
        case 'PDC'
            h = Controller.model.h;
            h = subs(h,'psi',State(8));
            h = double(h);
            
            U = [0;0;0;0];
            
            for i=1:length(K)
                U = U - K{i}*h(i)*(State - DesState);
                %this control law was designed for the
                %error dynamics so the 'state' it uses
                %is the error in position and velocity
            end
        case 'PDC_SOF'
            h = Controller.model.h;
            h = subs(h,'psi',State(8));
            h = double(h);
            
            U = [0;0;0;0];
            C = Controller.model.C;
            for i=1:length(K)
                U = U - K{i}*h(i)*C*(State - DesState);
                %this control law was designed for the
                %error dynamics so the 'state' it uses
                %is the error in position and velocity
            end
        otherwise
           U=zeros(4,length(t)); 
    end
end