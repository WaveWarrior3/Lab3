clear all
close all
clc

x = linspace(0,20,21);
time = linspace(0,20,20000);

v_L = [0,-5,-10,-10,-10,-5,0,5,10,10,10,5,0,-5,-10,-10,-10,-10,-10,-5,0];
v_R = [0,-5,-10,-10,-10,-10,-10,-10,-10,-10,-5,0,5,10,10,10,5,0,-5,-10,-10];

left = interp1(x,v_L,time);
right = interp1(x,v_R,time);

figure(1)
plotdefaults(16,5,2,'northeast');
plot(time,left)
hold on
plot(time, right)
legend('Left motor inputs','Right motor inputs', 'Location', 'NorthWest')
xlabel('Time (sec)');
ylabel('Motor velocity (rad/sec)')
tightfig(1)
saveas(gcf,'Complex_Inputs_2.pdf')


save('complex_inputs_2.mat', 'left', 'right');
A = [transpose(left) transpose(right)];
csvwrite('complex_inputs_2.csv', A)
