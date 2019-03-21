function [SAT_V] = saturate_control(V)
   
    SAT_V=V;
    
    max=1;min=-1;
    
    sat=find(SAT_V>max);
    for i = 1:length(sat)
        SAT_V(sat(i))=1;
    end
    
    
    sat=find(SAT_V<min);
    for i = 1:length(sat)
        SAT_V(sat(i))=-1;
    end
end

