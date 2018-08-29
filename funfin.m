function Ffin=myfun(x)

L1=.7;
L2=.5;
L3=1.3;

% Configuração Intermediária das Juntas (ponto B)
H=Trans(x(1),0,0)*Trans(0,0,L1)*Rot('y',x(2))*Trans(0,0,L2)*Rot('x',x(3))*Trans(0,0,L3);
Pfin=[.83;.19;-1.02];
Ffin=[H(1,4)-Pfin(1,1)
 H(2,4)-Pfin(2,1)
 H(3,4)-Pfin(3,1)];