function a2=acc2(output, input)

r5 = 292.5;
r6 = input(1);
r6dot = input(2);
w5 = input(3);
w6 = input(4);
theta5 = input(5);
theta6 = input(6);
a5 = input(7);


a2=[ -a5*r5*sin(theta5) - w5*w5*r5*cos(theta5) + output(2)*cos(theta6) - 2*r6dot*w6*sin(theta6) - output(1)*r6*sin(theta6) - w6*w6*r6*cos(theta6);
     a5*r5*cos(theta5) - w5*w5*r5*sin(theta5) + output(2)*sin(theta6) + 2*r6dot*w6*cos(theta6) + output(1)*r6*cos(theta6) - w6*w6*r6*sin(theta6)];
end