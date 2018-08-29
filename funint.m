function Fint=myfun(x)

L1=.7;
L2=.5;
L3=1.3;

% Configuração Intermediária das Juntas (ponto B)
H=Trans(x(1),0,0)*Trans(0,0,L1)*Rot('y',x(2))*Trans(0,0,L2)*Rot('x',x(3))*Trans(0,0,L3);
Pint=[1.31;.05;-.71];
Fint=[H(1,4)-Pint(1,1)
 H(2,4)-Pint(2,1)
 H(3,4)-Pint(3,1)];