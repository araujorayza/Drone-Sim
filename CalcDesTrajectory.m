function [q_d,dq_d,ddq_d]=CalcDesTrajectory(type,t)
    switch type
        case 'circle'
            raio = 0.5;
            w=0.3;
            q_d =   [raio*cos(w*t);
                    raio*sin(w*t);
                    ones(1,length(t));
                    zeros(1,length(t))];
            
            dq_d =  [-raio*w*sin(w*t);
                    raio*w*cos(w*t);
                    zeros(1,length(t));
                    zeros(1,length(t))];
            
            ddq_d = [-raio*w^2*cos(w*t);
                    -raio*w^2*sin(w*t);
                    zeros(1,length(t));
                    zeros(1,length(t))]; 
                
        case 'LemniscataBernoulli'
            a=1;
            
            q_d =  [a*cos(t).*sqrt(2.*cos(2*t));
                    a*sin(t).*sqrt(2.*cos(2*t));
                    ones(1,length(t));
                    zeros(1,length(t))];
            
            dq_d = [-(2^(1/2)*a.*sin(3.*t))./cos(2.*t).^(1/2);
                    (2^(1/2)*a.*cos(3.*t))./cos(2.*t).^(1/2);
                    zeros(1,length(t));
                    zeros(1,length(t))];
           
            ddq_d =[-(2^(1/2)*a.*(cos(5.*t) + 2.*cos(t)))./cos(2.*t).^(3/2);
                    -(2^(1/2)*a.*(sin(5.*t) + 2.*sin(t)))./cos(2.*t).^(3/2);
                    zeros(1,length(t));
                    zeros(1,length(t))];
        otherwise
            q_d     = zeros(4,length(t));
            dq_d    = zeros(4,length(t));
            ddq_d   = zeros(4,length(t));
    end
end