function w=position(output, input)
w=[210*cos(input*pi/180) + 400*cos(output(1)) - 500 - 430*cos(output(2));
     210*sin(input*pi/180) + 400*sin(output(1)) - 430*sin(output(2))];
end