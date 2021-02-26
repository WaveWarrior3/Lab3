
left = zeros([1,20000]);
right = zeros([1,20000]);
for i = 1:1000
    left(i) = -1/500*i;
    right(i) = 1/500*i;
end
for i = 1001:9500
    left(i) = -2;
    right(i) = 2;
end
for i = 9501:10500
    right(i) = -1/250*i+10000/250;
    left(i) = -abs(1/250*i-10000/250);
end
for i = 10501:20000
    left(i) = -2;
    right(i) = -2;
end

save('spin_straight_inputs.mat', 'left', 'right');

A = [left; right];
csvwrite('spin_straight_inputs.csv', A)
