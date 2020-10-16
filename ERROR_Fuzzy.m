function dERROR = ERROR_Fuzzy(t,ERROR,Simu)
    A = Simu.controller.model.A;
    B = Simu.controller.model.B;
    
    trajectory = Simu.trajectory;
    [q_d,dq_d,~]=CalcDesTrajectory(trajectory,t);
    DES_STATE = [dq_d;q_d];
    Psi = ERROR(8)+ DES_STATE(8);
    h = Simu.controller.model.h;
    h = subs(h,'psi',Psi);
    h = double(h);
    
         
    STATE = ERROR + DES_STATE; 
    V = CalcVirtControlLaw(Simu.controller,t,STATE,DES_STATE);

    if(Simu.controller.model.type == 2)
        Ah     = 0*A{1,1};
        Bh     = 0*B{1};
        for i=1:length(h)
            for j = 1:length(h)
                Ah=Ah+h(i)*h(j)*A{i,j};
            end
            Bh=Bh+h(i)*B{i};
        end 
    else
        Ah     = 0*A{1};
        Bh     = 0*B{1};
        for i=1:length(h)
            Ah=Ah+h(i)*A{i};
            Bh=Bh+h(i)*B{i};
        end
    end
    
    
    dERROR = Ah*ERROR + Bh*V;
end