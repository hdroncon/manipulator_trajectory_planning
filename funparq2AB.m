function Fq2AB=myfun3(x,qini,qint)


tf=2;
a0q2=qini(2,1); % posição inicial da junta
a1q2=0; % velocidade inicial da junta
pfjq2=qint(2,1); % posição final da junta
vfjq2=0; % velocidade final da junta


Fq2AB=[x(2)*(tf^3)+x(1)*(tf^2)+a1q2*tf+a0q2-pfjq2
    3*x(2)*(tf^2)+2*x(1)*tf+a1q2-vfjq2];