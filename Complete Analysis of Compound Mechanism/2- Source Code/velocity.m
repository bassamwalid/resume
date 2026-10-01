function v=velocity(output, input)

r1 = 210;
r2 = 400;
r3 = 500;
r4 = 430;
r5 = 292.5;
r7 = 361.5;
w1 = 10;

v=[ -w1*r1*sin(input(1)) - output(1)*r2*sin(input(2)) + output(2)*r4*sin(input(3));
     w1*r1*cos(input(1)) + output(1)*r2*cos(input(2)) - output(2)*r4*cos(input(3))];
end