%loads current gamma and resaves it in a different file
%with the modification date
load gamma gamma
fileName=strcat(num2str(clock)')'
fileName=strcat('gamma_',fileName);
fileName=strcat(fileName,'.mat');
save(fileName,'gamma');


clear gamma
gamma= [3.3264;
    0.1305;
    3.3508;
    0.1781;
    0.9710;
    1.0685;
    4.6840;
    2.7530];

save('gamma.mat','gamma');
clear gamma