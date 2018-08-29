function FH1AB=myfun3(x,qini,qint)


tf=2;
a0H1=qini(1,1); % posição inicial da junta
a1H1=0; % velocidade inicial da junta
pfjH1=qint(1,1); % posição final da junta
vfjH1=.4; % velocidade final da junta


FH1AB=[x(2)*(tf^3)+x(1)*(tf^2)+a1H1*tf+a0H1-pfjH1
    3*x(2)*(tf^2)+2*x(1)*tf+a1H1-vfjH1];
