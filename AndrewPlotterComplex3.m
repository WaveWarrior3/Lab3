clc; clear all;

t = linspace(0,20-20/2000,2000);

pydata = csvread('EE_complex_segway_3.csv');
dFpy = pydata(:,1) - 0.25;
dRpy = pydata(:,2) - 0.35;
dFpy = downsample(dFpy,10);
dRpy = downsample(dRpy,10);

load('Segway_Complex3_Data_Trial1.mat');
dF1 = distance_F_library/409.6;
dR1 = distance_R_library/409.6;
dF1 = downsample(dF1,10);
dR1 = downsample(dR1,10);

load('Segway_Complex3_Data_Trial2.mat');
dF2 = distance_F_library/409.6;
dR2 = distance_R_library/409.6;
dF2 = downsample(dF2,10);
dR2 = downsample(dR2,10);

load('Segway_Complex3_Data_Trial3.mat');
dF3 = distance_F_library/409.6;
dR3 = distance_R_library/409.6;
dF3 = downsample(dF3,10);
dR3 = downsample(dR3,10);

figure(1)
subplot(2,1,1)
plot(t,dF1,'.',t,dF2,'.',t,dF3,'.','MarkerSize',3); hold on;
plot(t,dFpy,'k','Linewidth',1.5)
title('Front Sensor')
xlabel('Time, {\it t} [s]')
ylabel('Lidar Distance [m]')
legend('Simulation 1','Simulation 2','Simulation 3','Model','location','best')
% plot(tEE,M(:,1)-.025,'k')
subplot(2,1,2)
plot(t,dR1,'.',t,dR2,'.',t,dR3,'.','MarkerSize',3); hold on;
plot(t,dRpy,'k','Linewidth',1.5)
title('Right Sensor')
xlabel('Time, {\it t} [s]')
ylabel('Lidar Distance [m]')

muF = mean([dF1; dF2; dF3]);
PEF = 100*abs(dFpy - muF')./((dFpy+muF')/2);
disp(mean(PEF))

muR = mean([dR1; dR2; dR3]);
PER = 100*abs(dRpy - muR')./((dRpy+muR')/2);
disp(mean(PER))

saveas(gcf,'Lab3_SegwayComplicated3_Lidar.pdf')