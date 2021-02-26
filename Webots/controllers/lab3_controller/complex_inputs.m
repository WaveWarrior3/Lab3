clear all
close all
clc

x = linspace(0,20,21);
time = linspace(0,20,20000);

v_L = [0,-1,-1.5,-1.5,-1.75,-1.75,-1.75,-2,-2,-2.25,-2.25,-2.25,-2.5,-2.5,-2,-1.5,-1,-1,-.5,0,0.5];
v_R = [0,-1,-2,-2,-2,-2,-2.5,-2.5,-2.5,-2,-2,-1.5,-1.5,-1.5,-1,-1,-.5,-.5,-.5,-1,-1];

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
saveas(gcf,'Complex_Input.pdf')


save('complex_inputs.mat', 'left', 'right');
A = [transpose(left) transpose(right)];
csvwrite('complex_inputs.csv', A)
