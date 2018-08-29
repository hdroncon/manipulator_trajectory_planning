function saida = Rot( eixo, theta)

if eixo=='x'
    saida = [1 0 0 0; 0 cos(theta) -sin(theta) 0 ; 0 sin(theta) cos(theta) 0; 0 0 0 1];
end
if eixo=='y'
    saida = [ cos(theta)  0 sin(theta) 0; 0 1 0 0; -sin(theta) 0 cos(theta) 0; 0 0 0 1];
end
if eixo=='z'
    saida = [ cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1];
end

