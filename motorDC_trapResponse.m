function [y, dy, t] = motorDC_trapResponse(p, K, u, t0, t1)
% func = K/(s+p)
num = K;
denom = [1,p];
sys = tf(num, denom);

t = 1:t0+t1;
u_trapecio = u .* ones(t0+t1,1);
u_trapecio(1:t0) = u/t0 * t(1:t0);

q = 48; %CPR
T = 1e-3; %Periodo de muestreo

[y, ~, ~] = lsim(sys, u_trapecio, t/1000);

y_extra = zeros(length(y)+1,1);
y_extra(2:end) = y;
y = [y; 0];
dy = 2*pi/q/T * (y-y_extra);

y=y(1:end-1);
dy(1)=0;
dy=dy(1:end-1);

%Descomentar para realizar el plot descrito en Script_E2.m
% figure(1)
% plot(y)
% figure(2)
% plot(dy)

end

