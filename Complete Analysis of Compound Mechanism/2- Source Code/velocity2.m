function v2=velocity2(output, input)

r1 = 210;
r2 = 400;
r3 = 500;
r4 = 430;
r5 = 292.5;
r7 = 361.5;
w1 = 10;

v2=[ -input(1)*r5*sin(input(2)) + output(1)*cos(input(3)) - output(2)*input(4)*sin(input(3));
      input(1)*r5*cos(input(2)) + output(1)*sin(input(3)) + output(2)*input(4)*cos(input(3))];
end