% Nombre del archivo
%%% PARA ESTE FICHERO TENEMOS DOS EJEMPLOS:
%%%     1. Kp=10  realizando 5 vueltas completas    %%%
%%%     2. Kp=0.8534 realizando 2 vueltas completas %%%

 % archivo = 'motorPos_K_10.txt';
 % Kp0 = 10;
 % targetPos = 5*2*pi;

archivo = 'motorPos_K_0.8534_1.txt';
Kp0 = 0.8534;
targetPos = 2*2*pi;
%%

% Abrir el archivo para lectura
fid = fopen(archivo, 'r');

% Omitir las primeras 4 líneas de cabecera
for i = 1:4
    fgetl(fid);
end

% Inicializar arrays para almacenar los datos
tiempo = [];
motorPos = [];

% Leer el resto del archivo línea por línea
while ~feof(fid)
    % Leer una línea del archivo
    line = fgetl(fid);
    
    % Extraer los datos numéricos después de "WRITE" y el tiempo en ms
    parts = strsplit(line, {'";"'});
    motorPos_str = parts{2}; % Datos motorPos como cadena
    tiempo_str = parts{end}; % Tiempo como cadena
    
    % Convertir los datos de motorPos a números
    motorPos_num = str2double(motorPos_str);
    
    % Convertir el tiempo a milisegundos
    if contains(tiempo_str, 'µs')
        tiempo_num = sscanf(tiempo_str, '%f') / 1000; % Convertir microsegundos a milisegundos
    elseif contains(tiempo_str, 'ms')
        tiempo_num = sscanf(tiempo_str, '%f'); % No hace falta conversión
    elseif contains(tiempo_str, 's')
        tiempo_num = sscanf(tiempo_str, '%f') * 1000; % Convertir segundos a milisegundos
    else
        error('Formato de tiempo no reconocido');
    end
    
    % Almacenar los datos en los arrays
    motorPos = [motorPos; motorPos_num];
    tiempo = [tiempo; tiempo_num];
end

% Guardar posición y tiempo de una K concreta
motorPosT = [tiempo, motorPos];
% motorPosT_str = strcat("\dataSTM_transf\motorPosT_", num2str(Kp));
% save motorPosT_str motorPosT;

% Cerrar el archivo
fclose(fid);

%% Plots
hold on
plot(motorPosT(:,1), motorPosT(:,2));
yline(targetPos, 'r');
legend("Posición obtenida", "Posición Objetivo")
ylabel("Posición (rad)")
xlabel("Tiempo (ms)")
hold off

%% Optimización función de transferencia a comportamiento motor
Kp0 = 10;
%Kp0 = 0.8534;
A = [];
B = [];

Kp_op = fmincon(@(Kp_op) myfun(Kp_op, motorPosT, x), Kp0, A, B);

H_op = tf(K*Kp_op, [1 p K*Kp_op]);
y = step(targetPos*H_op, t);
plot(y)
hold on
plot(motorPosT(1:2001,2))
legend("optimizada", "motor")
hold off

function metric = myfun(Kp_op, motorPosT, x)
K=x(2);
p=x(1);

H = tf(K*Kp_op, [1 p K*Kp_op]);
t=0:0.001:2;
respSis = step(targetPos* H, t);

metric = sqrt(mean((motorPosT(1:2001,2) - respSis).^2));

end










