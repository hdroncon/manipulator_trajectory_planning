% A4_test
clc
syms qH1 qq2 qq3 a3H1 a2H1 a3q3 a2q3 a3q2 a2q2 qH12 qq22 qq32
%qH1 qq2 qq3
%% Configuração do Robô
L1=.7;
L2=.5;
L3=1.3;

% Configuração Inicial das Juntas (ponto A)
Pini=[1.24;-.3;-.2];
qini=fsolve(@(x) funini(x),[0;0;0]);

% Configuração Intermediária das Juntas (ponto B)
Pint=[1.31;.05;-.71];
qint=fsolve(@(x) funint(x),[0;0;0]);

% Configuração final das Juntas (ponto C)
Pfin=[.83;.19;-1.02];
qfin=fsolve(@(x) funfin(x),[0;0;0]);


%% Planejamento de Trajetória & Cinemática 
% q=a3*t^3+a2*t^2+a1*t+a0
% q_d=3*a3*t^2+2*a2*t+a1
% q_dd=6*a3*t+2*a2
% qini=a0
% q_dini=a1
% qfin=a3*tf^3+a2*tf^2+a1*tf+a0
% q_dfin=3*a3*tf^2+2*a2*tf+a1

% A-B
% Alocação de variáveis
A=[1.24;-.3;-.2];
B=[1.31;.05;-.71];
HAB=Trans(0,qH1,0)*Trans(L1,0,0)*Rot('z',qq2)*Trans(L2,0,0)*Rot('y',qq3)*Trans(L3,0,0);
JAB=[diff(HAB(1,4),qH1) diff(HAB(1,4),qq2) diff(HAB(1,4),qq3);
     diff(HAB(2,4),qH1) diff(HAB(2,4),qq2) diff(HAB(2,4),qq3);
     diff(HAB(3,4),qH1) diff(HAB(3,4),qq2) diff(HAB(3,4),qq3)];
JABinv=inv(JAB);
JAB_d=[diff(JAB(1,1),qq2)+diff(JAB(1,1),qq3) diff(JAB(1,2),qq2)+diff(JAB(1,2),qq3) diff(JAB(1,3),qq2)+diff(JAB(1,3),qq3);
       diff(JAB(2,1),qq2)+diff(JAB(2,1),qq3) diff(JAB(2,2),qq2)+diff(JAB(2,2),qq3) diff(JAB(2,3),qq2)+diff(JAB(2,3),qq3);
       diff(JAB(3,1),qq2)+diff(JAB(3,1),qq3) diff(JAB(3,2),qq2)+diff(JAB(3,2),qq3) diff(JAB(3,3),qq2)+diff(JAB(3,3),qq3)];
TCPposABv=[];
TCPvelABv=[];
TCPacelABv=[];
qABv=[];
q_dABv=[];
q_ddABv=[];

% Equações das Juntas
% H1
a0H1=qini(1,1); % posição inicial da junta
a1H1=0; % velocidade inicial da junta
H1parAB=fsolve(@(x) funparH1AB(x,qini,qint),[0;0]);
a2H1=H1parAB(1,1);
a3H1=H1parAB(2,1);
% q2
a0q2=qini(2,1); % posição inicial da junta
a1q2=0; % velocidade inicial da junta
q2parAB=fsolve(@(x) funparq2AB(x,qini,qint),[0;0]);
a2q2=q2parAB(1,1);
a3q2=q2parAB(2,1);
% q3
a0q3=qini(3,1); % posição inicial da junta
a1q3=0; % velocidade inicial da junta
q3parAB=fsolve(@(x) funparq3AB(x,qini,qint),[0;0]);
a2q3=q3parAB(1,1);
a3q3=q3parAB(2,1);

for t=0:.01:2
    % JuntaH1
    qH1=a3H1*t^3+a2H1*t^2+a1H1*t+a0H1;
    q_dH1=3*a3H1*t^2+2*a2H1*t+a1H1;
    q_ddH1=6*a3H1*t+2*a2H1;
    % Juntaq2
    qq2=a3q2*t^3+a2q2*t^2+a1q2*t+a0q2;
    q_dq2=3*a3q2*t^2+2*a2q2*t+a1q2;
    q_ddq2=6*a3q2*t+2*a2q2;
    % Juntaq3
    qq3=a3q3*t^3+a2q3*t^2+a1q3*t+a0q3;
    q_dq3=3*a3q3*t^2+2*a2q3*t+a1q3;
    q_ddq3=6*a3q3*t+2*a2q3;
    % Agrupamento dos cálculos em um vetor 3x1
    qAB=[qH1;qq2;qq3];
    qABv=[qABv,qAB];
    q_dAB=[q_dH1;q_dq2;q_dq3];
    q_dABv=[q_dABv,q_dAB];
    q_ddAB=[q_ddH1;q_ddq2;q_ddq3];
    q_ddABv=[q_ddABv,q_ddAB];
    % TCP
    % Posição
    THAB=eval(HAB);
    TCPposAB=THAB(1:3,4);
    TCPposABv=[TCPposABv,TCPposAB];
    % Velocidade
    JacobAB=eval(JAB);
    TCPvelAB=JacobAB*q_dAB;
    TCPvelABv=[TCPvelABv,TCPvelAB];
    % Aceleração
    JacobAB=eval(JAB);
    JacobdAB=eval(JAB_d);
    TCPacelAB=(JacobAB*q_ddAB)+(JacobdAB*q_dAB);
    TCPacelABv=[TCPacelABv,TCPacelAB];
end

% B-C
% Alocação de variáveis
B=[1.31;.05;-.71];
C=[.83;.19;-1.02];
HBC=Trans(0,qH12,0)*Trans(L1,0,0)*Rot('z',qq22)*Trans(L2,0,0)*Rot('y',qq32)*Trans(L3,0,0);
JBC=[diff(HBC(1,4),qH12) diff(HBC(1,4),qq22) diff(HBC(1,4),qq32);
     diff(HBC(2,4),qH12) diff(HBC(2,4),qq22) diff(HBC(2,4),qq32);
     diff(HBC(3,4),qH12) diff(HBC(3,4),qq22) diff(HBC(3,4),qq32)];
JBC_d=[diff(JBC(1,1),qq22)+diff(JBC(1,1),qq32) diff(JBC(1,2),qq22)+diff(JBC(1,2),qq32) diff(JBC(1,3),qq22)+diff(JBC(1,3),qq32);
       diff(JBC(2,1),qq22)+diff(JBC(2,1),qq32) diff(JBC(2,2),qq22)+diff(JBC(2,2),qq32) diff(JBC(2,3),qq22)+diff(JBC(2,3),qq32);
       diff(JBC(3,1),qq22)+diff(JBC(3,1),qq32) diff(JBC(3,2),qq22)+diff(JBC(3,2),qq32) diff(JBC(3,3),qq22)+diff(JBC(3,3),qq32)];
JBCinv=inv(JBC);
TCPposBCv=[];
TCPvelBCv=[];
TCPacelBCv=[];
qBCv=[];
q_dBCv=[];
q_ddBCv=[];

% Equações das Juntas
% H1
a0H1=qint(1,1); % posição inicial da junta
a1H1=.4; % velocidade inicial da junta
H1parBC=fsolve(@(x) funparH1BC(x,qint,qfin),[0;0]);
a2H1=H1parBC(1,1);
a3H1=H1parBC(2,1);
% q2
a0q2=qint(2,1); % posição inicial da junta
a1q2=0; % velocidade inicial da junta
q2parBC=fsolve(@(x) funparq2BC(x,qint,qfin),[0;0]);
a2q2=q2parBC(1,1);
a3q2=q2parBC(2,1);
% q3
a0q3=qint(3,1); % posição inicial da junta
a1q3=0; % velocidade inicial da junta
q3parBC=fsolve(@(x) funparq3BC(x,qint,qfin),[0;0]);
a2q3=q3parBC(1,1);
a3q3=q3parBC(2,1);


for t=0:.01:2
    % JuntaH1
    qH12=a3H1*t^3+a2H1*t^2+a1H1*t+a0H1;
    q_dH1=3*a3H1*t^2+2*a2H1*t+a1H1;
    q_ddH1=6*a3H1*t+2*a2H1;
    % Juntaq2
    qq22=a3q2*t^3+a2q2*t^2+a1q2*t+a0q2;
    q_dq2=3*a3q2*t^2+2*a2q2*t+a1q2;
    q_ddq2=6*a3q2*t+2*a2q2;
    % Juntaq3
    qq32=a3q3*t^3+a2q3*t^2+a1q3*t+a0q3;
    q_dq3=3*a3q3*t^2+2*a2q3*t+a1q3;
    q_ddq3=6*a3q3*t+2*a2q3;
    % Agrupamento dos cálculos em um vetor 3x1
    qBC=[qH12;qq22;qq32];
    qBCv=[qBCv,qBC];
    q_dBC=[q_dH1;q_dq2;q_dq3];
    q_dBCv=[q_dBCv,q_dBC];
    q_ddBC=[q_ddH1;q_ddq2;q_ddq3];
    q_ddBCv=[q_ddBCv,q_ddBC];
    % TCP
    % Posição
    THBC=eval(HBC);
    TCPposBC=THBC(1:3,4);
    TCPposBCv=[TCPposBCv,TCPposBC];
    % Velocidade
    JacobBC=eval(JBC);
    TCPvelBC=JacobBC*q_dBC;
    TCPvelBCv=[TCPvelBCv,TCPvelBC];
    % Aceleração
    JacobBC=eval(JBC);
    JacobdBC=eval(JBC_d);
    TCPacelBC=(JacobBC*q_ddBC)+(JacobdBC*q_dBC);
    TCPacelBCv=[TCPacelBCv,TCPacelBC];
end

% Plotagem
time1=0:.01:2;
time2=2:.01:4;
% Posição
% Junta H1
figure (1)
subplot(1,3,1)      
plot(time1,qABv(1,:),time2,qBCv(1,:))          
title('Junta H1')
xlabel('Tempo [s]')
ylabel('Posição [m]')
% Junta q2
subplot(1,3,2)       
plot(time1,qABv(2,:),time2,qBCv(2,:))           
title('Junta q2')
xlabel('Tempo [s]')
ylabel('Posição [rad]')
% Junta q3
subplot(1,3,3)       
plot(time1,qABv(3,:),time2,qBCv(3,:))          
title('Junta q3')
xlabel('Tempo [s]')
ylabel('Posição [rad]')

% Velocidade
% Junta H1
figure (2)
subplot(1,3,1)      
plot(time1,q_dABv(1,:),time2,q_dBCv(1,:))         
title('Junta H1')
xlabel('Tempo [s]')
ylabel('Velocidade [m/s]')
% Junta q2
subplot(1,3,2)      
plot(time1,q_dABv(2,:),time2,q_dBCv(2,:))          
title('Junta q2')
xlabel('Tempo [s]')
ylabel('Velocidade [rad/s]')
% Junta q3
subplot(1,3,3)       
plot(time1,q_dABv(3,:),time2,q_dBCv(3,:))         
title('Junta q3')
xlabel('Tempo [s]')
ylabel('Velocidade [rad/s]')

% Aceleração
% Junta H1
figure (3)
subplot(1,3,1)      
plot(time1,q_ddABv(1,:),time2,q_ddBCv(1,:))         
title('Junta H1')
xlabel('Tempo [s]')
ylabel('Aceleração [m/s^2]')
% Junta q2
subplot(1,3,2)      
plot(time1,q_ddABv(2,:),time2,q_ddBCv(2,:))          
title('Junta q2')
xlabel('Tempo [s]')
ylabel('Aceleração [rad/s^2]')
% Junta q3
subplot(1,3,3)      
plot(time1,q_ddABv(3,:),time2,q_ddBCv(3,:))          
title('Junta q3')
xlabel('Tempo [s]')
ylabel('Aceleração [rad/s^2]')

% TCP
% Posição
figure (4)
subplot(1,3,1)       
plot(time1,TCPposABv(1,:),time2,TCPposBCv(1,:))          
title('Posição TCP eixo X')
xlabel('Tempo [s]')
ylabel('Posição [m]')
subplot(1,3,2)       
plot(time1,TCPposABv(2,:),time2,TCPposBCv(2,:))          
title('Posição TCP eixo Y')
xlabel('Tempo [s]')
ylabel('Posição [m]')
subplot(1,3,3)       
plot(time1,TCPposABv(3,:),time2,TCPposBCv(3,:))          
title('Posição TCP eixo Z')
xlabel('Tempo [s]')
ylabel('Posição [m]')

% Velocidade
figure (5)
subplot(1,3,1)       
plot(time1,TCPvelABv(1,:),time2,TCPvelBCv(1,:))          
title('Velocidade TCP eixo X')
xlabel('Tempo [s]')
ylabel('Velocidade [m/s]')
subplot(1,3,2)       
plot(time1,TCPvelABv(2,:),time2,TCPvelBCv(2,:))          
title('Velocidade TCP eixo Y')
xlabel('Tempo [s]')
ylabel('Velocidade [m/s]')
subplot(1,3,3)       
plot(time1,TCPvelABv(3,:),time2,TCPvelBCv(3,:))          
title('Velocidade TCP eixo Z')
xlabel('Tempo [s]')
ylabel('Velocidade [m/s]')

% Aceleração
figure (6)
subplot(1,3,1)       
plot(time1,TCPacelABv(1,:),time2,TCPacelBCv(1,:))          
title('Aceleração TCP eixo X')
xlabel('Tempo [s]')
ylabel('Aceleração [m/s^2]')
subplot(1,3,2)       
plot(time1,TCPacelABv(2,:),time2,TCPacelBCv(2,:))          
title('Aceleração TCP eixo Y')
xlabel('Tempo [s]')
ylabel('Aceleração [m/s^2]')
subplot(1,3,3)       
plot(time1,TCPacelABv(3,:),time2,TCPacelBCv(3,:))          
title('Aceleração TCP eixo Z')
xlabel('Tempo [s]')
ylabel('Aceleração [m/s^2]')