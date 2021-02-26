clear all
close all
clc

x = linspace(0,20,21);
time = linspace(0,20,20000);

v_L = [-10,-10,-10,-10,-10,-10,0,0,0,0,0,0,0,10,10,10,10,-10,-10,-10,-10];
v_R = [-10,-10,-10,-10,-10,-10,0,0,0,0,0,0,0,10,10,10,10,-10,-10,-10,-10];

left = interp1(x,v_L,time);
right = interp1(x,v_R,time);

figure(1)
plotdefaults(16,5,2,'northeast');
plot(time,left)
hold on
plot(time, right,'--')
legend('Left motor inputs','Right motor inputs', 'Location', 'NorthWest')
xlabel('Time (sec)');
ylabel('Motor velocity (rad/sec)')
tightfig(1)
saveas(gcf,'Complex_Inputs_3.pdf')


save('complex_inputs_3.mat', 'left', 'right');
A = [transpose(left) transpose(right)];
csvwrite('complex_inputs_3.csv', A)
