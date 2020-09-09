function [Ah,Bh] = Defuzzy(A,B,h,option)
    if(option == 1)
        Ah     = 0*A{1,1};
        Bh     = 0*B{1};
        for i=1:length(h)
            for j = 1:length(h)
                Ah=Ah+h(i)*h(j)*A{i,j};
            end
            Bh=Bh+h(i)*B{i};
        end 
    elseif(option == 3)
        Ah     = 0*A{1};
        Bh     = B{1};
        for i=1:length(h)
            Ah=Ah+h(i)*A{i};
        end
    else
        Ah     = 0*A{1};
        Bh     = 0*B{1};
        for i=1:length(h)
            Ah=Ah+h(i)*A{i};
            Bh=Bh+h(i)*B{i};
        end
    end

end 