h=1;
%%%%%%%%%%%%%%%%%TRAJECTORY PLOT%%%%%%%%%%%%%%%%%%%%%%%
figure(h);

hold on;
plot(STATE(:,PosX),STATE(:,PosY),'*');
plot(ERROR(:,PosX)+ DES_STATE(:,PosX),ERROR(:,PosY) + DES_STATE(:,PosY),'rd');
plot(DES_STATE(:,PosX),DES_STATE(:,PosY),'k*');
hold off
grid on
axis equal
legend('Trajectory')
legend('NonLinSys','FuzzyErrorSys','Ref')
h=h+1;

%%%%%%%%%%%%%%%%%CONTROL SIGNAL PLOT%%%%%%%%%%%%%%%%%%%%%%%
% figure(h);
% U = zeros(4,length(t));
% V = U;
% 
% for i = 1:length(t)
%     U(:,i) = CalcVirtControlLaw(ControllerStruct,t(i),STATE(i,:)',ddq_d(:,i),dq_d(:,i),q_d(:,i));
% %     V(:,i)=M\(U(:,i) + N*R'*dq_d + ddq_d);
% %     
% %     if (controller.sat)
% %         V=saturate_control(V);
% %     end
% end 
% 
% plot(t',U,'d');
% grid on
% title('Control Signal')
% legend('U_x','U_y','U_z','U_y_a_w');
% h=h+1;



%%%%%%%%%%%%%%%%%COMPONENT PLOT%%%%%%%%%%%%%%%%%%%%%%%
figure(h);
hold on
plot(t,STATE(:,PosX),'*');
plot(t,ERROR(:,PosX)+ DES_STATE(:,PosX),'rd');
plot(t,DES_STATE(:,PosX),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('x(t)')
h=h+1;

figure(h);
hold on
plot(t,STATE(:,PosY),'*');
plot(t,ERROR(:,PosY)+ DES_STATE(:,PosY),'rd');
plot(t,DES_STATE(:,PosY),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('y(t)')
h=h+1;

figure(h);
hold on
plot(t,STATE(:,PosZ),'*');
plot(t,ERROR(:,PosZ)+ DES_STATE(:,PosZ),'rd');
plot(t,DES_STATE(:,PosZ),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('z(t)')
h=h+1;

figure(h);
hold on
plot(t,STATE(:,PosYaw),'*');
plot(t,ERROR(:,PosYaw)+ DES_STATE(:,PosYaw),'rd');
plot(t,DES_STATE(:,PosYaw),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('yaw(t)')
h=h+1;

if(SimStruct.WindDisturbance.Type)
    figure(h);
    D = zeros(4,length(t));

    for i = 1:length(t)
        D(:,i) = WindDisturbance(t(i),SimStruct.WindDisturbance);
    end 

    plot(t',D(1,:));
    grid on
    % legend('$D_x$','$D_y$','$D_z$','$D_{\psi}$');
    legend('Disturbance Signal')
    h=h+1;
end 

%%%%%%%%%%%%%%%%%ERROR PLOT%%%%%%%%%%%%%%%%%%%%%%%
figure(h);
plot(t,ERROR,'d');
grid on
title('Error')
h=h+1;


%%%%%%%%%%%%%%%%%COMPONENT  VELOCITY PLOT%%%%%%%%%%%%%%%%%%%%%%%
figure(h);
hold on
plot(t,STATE(:,VelX),'*');
plot(t,ERROR(:,VelX)+ DES_STATE(:,VelX),'rd');
plot(t,DES_STATE(:,VelX),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('Velx(t)')
h=h+1;

figure(h);
hold on
plot(t,STATE(:,VelY),'*');
plot(t,ERROR(:,VelY)+ DES_STATE(:,VelY),'rd');
plot(t,DES_STATE(:,VelY),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('Vely(t)')
h=h+1;

figure(h);
hold on
plot(t,STATE(:,VelZ),'*');
plot(t,ERROR(:,VelZ)+ DES_STATE(:,VelZ),'rd');
plot(t,DES_STATE(:,VelZ),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('Velz(t)')
h=h+1;

figure(h);
hold on
plot(t,STATE(:,VelYaw),'*');
plot(t,ERROR(:,VelYaw)+ DES_STATE(:,VelYaw),'rd');
plot(t,DES_STATE(:,VelYaw),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('Velyaw(t)')
h=h+1;



