close all;

VelX    = 1;
VelY    = 2;
VelZ    = 3;
VelYaw  = 4;
PosX    = 5;
PosY    = 6;
PosZ    = 7;
PosYaw  = 8;



PATH='/home/lac/Dropbox/Aplicativos/ShareLaTeX/2020ICUAS/Figs/sim/teo2/';
h=1;
%%%%%%%%%%%%%%%%%TRAJECTORY PLOT%%%%%%%%%%%%%%%%%%%%%%%
figure(h);

hold on;
plot(STATE(:,PosX),STATE(:,PosY),'d');
plot(DES_STATE(:,PosX),DES_STATE(:,PosY),'k*-');
hold off
xlim([-0.33 0.33])
ylim([-0.33 0.33])
axis square
legend('Sys','Ref')
xlabel('$x$ (m)')
ylabel('$y$ (m)')

file=strcat(PATH,'traj.tex')
matlab2tikz(file,'width','0.5\textwidth','extraTikzpictureOptions','scale=0.9','parseStrings',false);

%%%%%%%%%%%%%%%%%CONTROL SIGNAL PLOT%%%%%%%%%%%%%%%%%%%%%%%
h=h+1;
figure(h);
U = zeros(4,length(t));
V = U;

for i = 1:length(t)
    U(:,i) = CalcVirtControlLaw(ControllerStruct,t(i),STATE(i,:)',ddq_d(:,i),dq_d(:,i),q_d(:,i));
%     V(:,i)=M\(U(:,i) + N*R'*dq_d + ddq_d);
%     
%     if (controller.sat)
%         V=saturate_control(V);
%     end
end 

plot(t',U);
% grid on
legend('$U_x$','$U_y$','$U_z$','$U_{\psi}$');
xlabel('$t$ (s)')
ylabel('Control Signal')

file=strcat(PATH,'control.tex')
matlab2tikz(file,'width','0.5\textwidth','extraTikzpictureOptions','scale=0.9','parseStrings',false);


%%%%%%%%%%%%%%%%%COMPONENT PLOT%%%%%%%%%%%%%%%%%%%%%%%
h=h+1;
figure(h);
hold on
plot(t,STATE(:,PosX),'*');
plot(t,DES_STATE(:,PosX),'k*-');
hold off
legend('Sys','Ref')
xlabel('$t$ (s)')
ylabel('$x$ (m)')
xlim([0 17.3])
ylim([-0.33 0.33])
axis square

file=strcat(PATH,'x.tex')
matlab2tikz(file,'width','0.5\textwidth','extraTikzpictureOptions','scale=0.9','parseStrings',false);


h=h+1;
figure(h);
hold on
plot(t,STATE(:,PosY),'*');
plot(t,DES_STATE(:,PosY),'k*-');
hold off
legend('Sys','Ref')
xlabel('$t$ (s)')
ylabel('$y$ (m)')
xlim([0 17.3])
ylim([-0.33 0.4])
axis square

file=strcat(PATH,'y.tex')
matlab2tikz(file,'width','0.5\textwidth','extraTikzpictureOptions','scale=0.9','parseStrings',false);

h=h+1;
figure(h);
hold on
plot(t,STATE(:,PosZ),'*');
plot(t,DES_STATE(:,PosZ),'k*-');
hold off
legend('Sys','Ref')
xlabel('$t$ (s)')
ylabel('$z$ (m)')
xlim([0 17.3])
ylim([-0.05 1.15])
axis square

file=strcat(PATH,'z.tex')
matlab2tikz(file,'width','0.5\textwidth','extraTikzpictureOptions','scale=0.9','parseStrings',false);

h=h+1;
figure(h);
hold on
plot(t,STATE(:,PosYaw),'*');
plot(t,DES_STATE(:,PosYaw),'k*-');
hold off
legend('Sys','Ref')
xlabel('$t$ (s)')
ylabel('$\psi$ (m)')
xlim([0 17.3])
% ylim([-1e-9 1e-9])
 ylim([-0.05 0.11])
% axis tight

file=strcat(PATH,'yaw.tex')
matlab2tikz(file,'width','0.5\textwidth','height','0.1\textwidth','extraTikzpictureOptions','scale=0.9','parseStrings',false);


%%%%%%%%%%%%%%%%%DISTURBANCE PLOT%%%%%%%%%%%%%%%%%%%%%%%
if(SimStruct.WindDisturbance.Type)
    h=h+1;
    figure(h);
    D = zeros(4,length(t));

    for i = 1:length(t)
        D(:,i) = WindDisturbance(t(i),SimStruct.WindDisturbance)
    end 

    plot(t',D(1,:));
    % grid on
    % legend('$D_x$','$D_y$','$D_z$','$D_{\psi}$');
    legend('Disturbance Signal')
    xlabel('$t$ (s)')
    ylabel('$\omega$ (m/s)')    
    ylim([-0.045 0.17])
    % ylabel('Disturbance Signal')

    file=strcat(PATH,'disturbance.tex')
    matlab2tikz(file,'width','0.5\textwidth','extraTikzpictureOptions','scale=0.9','parseStrings',false);
end 