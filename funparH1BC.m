function FH1BC=myfun3(x,qint,qfin)


tf=2;
a0H1=qint(1,1); % posição inicial da junta
a1H1=.4; % velocidade inicial da junta
pfjH1=qfin(1,1); % posição final da junta
vfjH1=0; % velocidade final da junta


FH1BC=[x(2)*(tf^3)+x(1)*(tf^2)+a1H1*tf+a0H1-pfjH1
    3*x(2)*(tf^2)+2*x(1)*tf+a1H1-vfjH1];
