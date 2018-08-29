function Fq3AB=myfun3(x,qini,qint)


tf=2;
a0q3=qini(3,1); % posição inicial da junta
a1q3=0; % velocidade inicial da junta
pfjq3=qint(3,1); % posição final da junta
vfjq3=0; % velocidade final da junta


Fq3AB=[x(2)*(tf^3)+x(1)*(tf^2)+a1q3*tf+a0q3-pfjq3
    3*x(2)*(tf^2)+2*x(1)*tf+a1q3-vfjq3];