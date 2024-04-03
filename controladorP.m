load("ControlSystemDesignerSession_Kp_0.8534.mat")
%%% Para el uso de este fichero previamente se ha de utilizar el .mat 
%%% cargado previamente y la herramienta "sisotool". %%%
%%% Una vez abierta, se abrirá ControlSystemDesignerSession en el botón
%%% "Open Session" para acceder a los datos contenidos.%%%
%%% Finalmente al exportar usando "Export" obtendremos los valores
%%% necesarios para poder utilizar este fichero. %%%

%%
% Controlador POSICIÓN
Kp = C.K;
sys=tf(IOTransfer_r2y);
excit=2*2*pi;

%% Controlador respuesta a escalón
figure(1)
step(excit*sys)

%% Controlador respuesta a rampa

t = 1:t0+t1;
u_trapecio = u .* ones(t0+t1,1);
u_trapecio(1:t0) = u/t0 * t(1:t0);

[y, ~, ~] = lsim(sys, u_trapecio, t/1000);

figure(2)
plot(y)
hold on
plot(u_trapecio)
hold off


