function w2=position2(output, input)

w2=[ 292.5*cos(input*pi/180) + output(1)*cos(output(2)) - 361.5 ;
     292.5*sin(input*pi/180) + output(1)*sin(output(2)) ];

end