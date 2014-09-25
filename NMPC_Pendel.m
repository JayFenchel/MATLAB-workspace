function NMPC_Pendel
%% Parameter
p.g = 9.80665;                      % Erdbeschleunigung in [m/(s*s)]
p.l = 0.5;                          % Laenge des Pendels in [m]
p.dt = 0.05;                          % Intervall der Messzeiten [s]

tspan = [0:p.dt:10];
delta = 0.02; % Verz�gerung durch Berechnung
N=10;   % Anzahl pr�dizierter Schritte


%% Anfangsbedingungen
Phi0=10*(2*pi/360);                % Anfangsauslenkung des Pendels
%5*(2*pi/360);                    % Beispiel fuer 5 Grad Auslenkung
x0=[0 0 Phi0 0];                  % Anfangsschaetzung Phi=0 , Phi_Punkt=0

%% Sollgr��en
x3_soll=0;

%% Eingangsvektor
u0=zeros(1,N+1)   % Startwert f�r die erste Sch�tzung
%%
[t_mess, x_mess] = ode45(@nmpcPendelDGS, tspan(1:40+1), x0, odeset, zeros(1,40+1), p);

figure(1)
subplot(2,1,1)
plot(t_mess,zeros(1,40+1));
subplot(2,1,2)
plot(t_mess,x_mess(:,3))
grid on

options = optimoptions('fminunc','Algorithm','quasi-newton');
[u_opt, fehlersum_opt] = fminunc(@(u) kosten(u, tspan(1:1+N), x0, x3_soll, p), u0);

%% Optimum plotten
[t_opt, x_opt] = ode45(@nmpcPendelDGS, tspan(1:1+N), x0, odeset, u_opt, p);

figure(2)
subplot(2,1,1)
plot(sort([t_opt;t_opt+p.dt]),reshape([u_opt;u_opt],1,length(u_opt)*2));
subplot(2,1,2)
plot(t_opt,x_opt(:,3))
grid on

x_mess = x0;
u_done=0;
for i=1:1:40
i
    %% Optimierung
    
    options = optimoptions('fminunc','Algorithm','quasi-newton');
%     options = optimoptions('fminunc','Algorithm','trust-region');
    [u_opt, fehlersum_opt] = fminunc(@(u) kosten(u, tspan(1:1+N), x_mess(end,:), x3_soll, p), u0, options);

    u_done = [u_done u_opt(2)]
    %% Messung
    [t_mess, x_mess] = ode45(@nmpcPendelDGS, tspan(1:i+1)-delta, x0, odeset, u_done, p);
end

%% Optimum plotten
figure(2)
subplot(2,1,1)
plot(sort([t_mess;t_mess-p.dt+delta]),reshape([u_done,u_done],1,length(u_done)*2));
subplot(2,1,2)
plot(t_mess,x_mess(:,3))
grid on

end

function J = kosten(u, tspan, x0, x3_soll, p)

[t_pred, x_pred] = ode45(@nmpcPendelDGS, tspan, x0, odeset, u, p);
J = sum((x_pred(:,3)-x3_soll).^2)
end


function dx = nmpcPendelDGS(t, x, u, p)

u = u(ceil(t/p.dt)+1);

dx(1) = x(2);
dx(2) = u;
dx(3) = x(4);
dx(4) = u/p.l * cos(x(3)) + p.g/p.l * sin(x(3)) - x(4)*0.1; % -x(4)*Faktor = Reibungsterm

dx = dx';

end