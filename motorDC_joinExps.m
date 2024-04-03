function [metric] = motorDC_joinExps(mediasDatos, p, K, t0, t1, signals)


% Posición
pos_reales_conj = cat(1, mediasDatos(:,1), mediasDatos(:,2),mediasDatos(:,3),mediasDatos(:,4), ...
    mediasDatos(:,5),mediasDatos(:,6),mediasDatos(:,7),mediasDatos(:,8), ...
    mediasDatos(:,9),mediasDatos(:,10),mediasDatos(:,11),mediasDatos(:,12));

for i = 1:signals
    [y, dy, t] = motorDC_trapResponse(p,K,i,t0,t1);
    if i==1
        pos_ideales_conj = y;
        vel_ideales_conj = dy;
    else
    pos_ideales_conj = cat(1, pos_ideales_conj, y);
    vel_ideales_conj = cat(1, vel_ideales_conj, dy);
    end
end

% Aplicar reductora para escalar encoder a motor
red = 464/48;
pos_reales_conj = pos_reales_conj / red;


% Velocidad
q = 48;
T=1e-3;
cte = 2*pi/q/T;

dy_mediasDatos =  cte.* ([mediasDatos ; zeros(1, signals)]-[zeros(1, signals) ; mediasDatos]);
dy_mediasDatos = dy_mediasDatos(1:end-1,:);

vel_reales_conj = cat(1, dy_mediasDatos(:,1), dy_mediasDatos(:,2),dy_mediasDatos(:,3),dy_mediasDatos(:,4), ...
    dy_mediasDatos(:,5),dy_mediasDatos(:,6),dy_mediasDatos(:,7),dy_mediasDatos(:,8), ...
    dy_mediasDatos(:,9),dy_mediasDatos(:,10),dy_mediasDatos(:,11),dy_mediasDatos(:,12));

% Error
metric = sqrt(mean((pos_reales_conj - pos_ideales_conj).^2));

% Plots
plot(pos_reales_conj)
hold on
plot(pos_ideales_conj)
xlabel("Tiempo (ms)")
ylabel("Pulsos encoder")
legend("Posición modelo real", "Posición modelo ideal")
hold off

end

