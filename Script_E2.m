clear, clc
%% CONEXIÓN PUERTO SERIE

% Crear un objeto de puerto serie
s = serialport("COM3", 115200); 

N_EXP = 12; %P
N_EXCIT = 12; %Q

% Definir el tamaño del array para almacenar las líneas
numLineas = 10000; 
tamano_datos = readline(s); % Leer una línea desde el puerto serie
%arrayDatos = cell(100000,1);
matrizDatos = zeros(1200,N_EXP,N_EXCIT);

% Leer datos del puerto serie y almacenar en el array
for k = 1:N_EXCIT
    fprintf("\nEXCITACIÓN: %d (V)\n",k);
    for j = 1:N_EXP
        fprintf("EXPERIMENTO: %d\n",j);
        for i = 1:100000
            linea = readline(s);
            if (linea=="end")
                break;
            end
            matrizDatos(i,j,k) = str2double(linea);
        end
    end
end

% Cerrar el puerto serie
delete(s);
clear s;

%% OBTENER MEDIAS A PARTIR DE DATOS RECIBIDOS

mediasDatos = mean(matrizDatos(:,2:12,:),2);
mediasDatos = squeeze(mediasDatos);

%% OBTENCIÓN MODELO EXPERIMENTAL MOTOR DC MINIMIZANDO DIFERENCIA ENTRE SALIDA REAL E IDEAL

x0 = [68.445, 1170.703]; %Valores teóricos de p y K
A = [];
B = [];
t0 = 600;
t1 = 600;
signals = 12;

% Creamos función anónima para insertar más variables a función myfun
myfun_anonima = @(x) myfun(x, t0, t1, signals, mediasDatos);

% Llamar a fmincon con la función anónima
x = fmincon(myfun_anonima, x0, A, B);


%x = fmincon(@myfun, x0, A, B);
function metric = myfun(x, t0, t1, signals, mediasDatos)
p=x(1);
K=x(2);

metric = motorDC_joinExps(mediasDatos, p, K, t0, t1, signals);

end

%% PLOTS
% POSICIÓN REAL
plot(mediasDatos)
ylabel("Pulsos del encoder")
xlabel("Tiempo (ms)")

% VELOCIDAD REAL
q = 48; %CPR
T = 1e-3; %Periodo de muestreo
dy = zeros(1200,12);
for i = 1:12
    for j = 1:1200
        if j == 1
            dif = mediasDatos(j,i);
        else
            dif = mediasDatos(j,i)-mediasDatos(j-1,i);
        end 
        dy(j,i) = 2*pi/q/T * dif;
    end
end
plot(dy)
ylabel("Velocidad angular (rad/s)")
xlabel("Tiempo (ms)")

% POSICIÓN Y VELOCIDAD IDEAL
% for i = 12:-1:0
%     hold on
%     motorDC_trapResponse(x(1), x(2), i, 600, 600);
%     hold off
% end
% figure(1)
% ylabel("Pulsos encoder")
% xlabel("Tiempo (ms)")
% figure(2)
% ylabel("Velocidad angular (rad/s)")
% xlabel("Tiempo (ms)")


