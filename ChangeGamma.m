%loads current gamma and resaves it in a different file
%with the modification date
load gamma gamma
fileName=strcat(num2str(clock)')'
fileName=strcat('gamma_',fileName);
fileName=strcat(fileName,'.mat');
save(fileName,'gamma');


clear gamma
% gamma= [3.3264;
%     0.1305;
%     3.3508;
%     0.1781;
%     0.9710;
%     1.0685;
%     4.6840;
%     2.7530];

%gamma identified on .yaml Sphinx (don't know the date of the experiment,
%probably march of 2019) 
gamma = [-0.2630   -0.1270   -0.3034   -0.1594    0.0502    0.2888   -0.0066   -0.0079]';
% gamma = [3.2849,   -0.5905,    3.3803,   -0.5075,    2.7960,    3.2259,    2.1494,    1.3987]';

%gamma measured on Feb 26th at CROB
% gamma = [4.0199    0.2033    3.5186    0.2037    0.2923   -0.1324    0.2417   -0.0458]';

%gamma identified on Feb 26th Sphinx
% gamma = [-0.1800   -0.0618   -0.1995   -0.0936    0.0225    0.0653   -0.0064   -0.0143]';

save('gamma.mat','gamma');
clear gamma