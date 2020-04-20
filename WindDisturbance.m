function W = WindDisturbance(t,WindDistStr)
%WINDDISTURBANCE Summary of this function goes here
%   Detailed explanation goes here
    Ws = WindDistStr.Ws;
    a = WindDistStr.a;
    Omegas = WindDistStr.Omegas;
    Gammas = WindDistStr.Gammas;
    StartTime = WindDistStr.time(1);
    EndTime = WindDistStr.time(end);
    
    W = zeros(4,1);
    
    if(t >= StartTime && t <= EndTime)
        for i = 1:length(a)
            W = W + 0.1*a(i)*sin(Omegas(i)*t + Gammas(i));
        end

        W = W + Ws;
    end 
   
end

