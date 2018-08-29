function Fini=myfun1(x)

L1=.7;
L2=.5;
L3=1.3;

% Configuração Inicial (ponto A)
H=Trans(x(1),0,0)*Trans(0,0,L1)*Rot('y',x(2))*Trans(0,0,L2)*Rot('x',x(3))*Trans(0,0,L3);
Pini=[1.24;-.3;-.2];
Fini=[H(1,4)-Pini(1,1)
 H(2,4)-Pini(2,1)
 H(3,4)-Pini(3,1)];