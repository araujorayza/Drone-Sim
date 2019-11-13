c=2;

x=1;y=2;z=3;psi=4;
PASTA_DA_FIG='/home/lac/Dropbox/Aplicativos/ShareLaTeX/phd-project/Figuras/OL'

err_f = ERROR(:,5:8);
err_nl = STATE(:,5:8) - DES_STATE(:,5:8);

h=figure(c);
plot(err_f(:,x),'d')
hold on
plot(err_nl(:,x),'-')
legend('x_f_u_z_z_y','x_N_L')
ylabel('x (m)')
xlabel('t (s)')
grid on
axis image
axis normal
FIGNAME=strcat(PASTA_DA_FIG,'/x')
print(h,FIGNAME,'-dpdf')
c=c+1;

h=figure(c);
plot(err_f(:,y),'d')
hold on
plot(err_nl(:,y),'-')
legend('y_f_u_z_z_y','y_N_L')
ylabel('y (m)')
xlabel('t (s)')
grid on
axis image
axis normal
FIGNAME=strcat(PASTA_DA_FIG,'/y')
print(h,FIGNAME,'-dpdf')
c=c+1;

h=figure(c);
plot(err_f(:,z),'d')
hold on
plot(err_nl(:,z),'-')
legend('z_f_u_z_z_y','z_N_L')
ylabel('z (m)')
xlabel('t (s)')
grid on
axis image
axis normal
FIGNAME=strcat(PASTA_DA_FIG,'/z')
print(h,FIGNAME,'-dpdf')
c=c+1;

h=figure(c);
plot(err_f(:,psi),'d')
hold on
plot(err_nl(:,psi),'-')
legend('psi_f_u_z_z_y','psi_N_L')
ylabel('psi (rad)')
xlabel('t (s)')
grid on
axis image
axis normal
FIGNAME=strcat(PASTA_DA_FIG,'/psi')
print(h,FIGNAME,'-dpdf')
c=c+1;