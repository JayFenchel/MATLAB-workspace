function ProbeNMPC
%% Bereinigung

clc; close all; clear all;
%% Parameter
g = 9.80665;                      % Erdbeschleunigung in [m/(s*s)]
l = 0.5;                          % Laenge des Pendels in [m]
T = 0.1;                          % Intervall der Messzeiten [s]
%% Anfangsbedingungen

Phi0=5*(2*pi/360);                % Anfangsauslenkung des Pendels
%5*(2*pi/360);                    % Beispiel fuer 5 Grad Auslenkung
x0=[0 0 Phi0 0];                  % Anfangsschaetzung Phi=0 , Phi_Punkt=0
tspan = [0 10];

% %% Zustandsraumdarstellung Pendel
% A = [0 1                          % Systemmatrix (linearisiertes Modell)
%     g/l 0];
% B = [0 1/l]';                     % Eingangsvektor
% %% Zustandsraumdarstellung Robo
% Ar = [0 1                         % Systemmatrix (linearisiertes Modell)
%     0 0];
% Br = [0 1]';                      % Eingangsvektor
% %% LQR-Regler
% 
% Q_Pendel = 0.0001 * eye(length(A));         % Gewichtungsmatrizen Q und R
% R_Pendel = 0.0001 * eye(1);                 % weil nur 1 Stellgroesse
% 
% Q_Robo = 0.0001 * eye(length(A));
% R_Robo = 0.001 * eye(1);
% 
% [K_Pendel] = lqr(A,B,Q_Pendel,R_Pendel);    % Reglerentwurf
% [K_Robo] = lqr(Ar,Br,Q_Robo,R_Robo);
% 
% %% Loesung des Differentialgleichungssystems mit LQ-Regler
% [t,x]=ode45(@PendelDGS,tspan,x0,odeset,K_Pendel,K_Robo,l,g);
% 
% t_n = tspan(1);
% x_n = x0;
% u_n = [];
% 
% while(t_n(end) < tspan(end) - T)
%     u = (x_n(end,1) * K_Robo(1) + x_n(end,2) * K_Robo(2)) -...
%         (x_n(end,3) * K_Pendel(1) + x_n(end,4) * K_Pendel(2));
%     
%     [t_hilf,x_hilf]=ode45(@DiscretePendelDGS,[t_n(end) t_n(end) + T],...
%         x_n(end,:), odeset,l,g,u);
%     
%     t_n = [t_n; t_hilf];
%     x_n = [x_n; x_hilf];
%     u_n = [u_n; u];
% end
% 
% %% Grafische Ausgabe
% figure('Name','Position und Geschwindigkeit des Robbi')
% plot(t(:),x(:,1))
% hold on
% plot(t(:),x(:,2),'r')
% plot(t_n(:),x_n(:,1),'k')
% plot(t_n(:),x_n(:,2),'g')
% plot(t_n(1):T:t_n(end),u_n,'m')
% title('Weg und Geschwindigkeit in x-Richtung')
% xlabel('Zeit [s]')
% ylabel('Position x [m]')
% legend('Weg kont.','Geschwindigkeit kont.','Weg diskret',...
%     'Geschwindigkeit diskret','Beschleunigung diskret')
% hold off
% 
% figure('Name','Auslenkung und Geschwindigkeit des Pendels')
% plot(t(:),x(:,3))
% hold on
% plot(t(:),x(:,4),'r')
% plot(t_n(:),x_n(:,3),'k')
% plot(t_n(:),x_n(:,4),'g')
% title('Auslenkung und Geschwindigkeit in x-Richtung')
% xlabel('Zeit [s]')
% ylabel('Auslenkung Phi [Grad]')
% hold off

%% Loesung des Differentialgleichungssystems mit NMPC

warning off all                 % alle Warnungen des Optimierers ausblenden

% Variablen initialisieren
t_mpc = [];
x_mpc = [];
u_mpc = [];
t_mess = tspan(1);
x_mess = x0;

N = 10;                          % Angabe, wie weit die Optimierung in die 
                                % Zukunft schauen soll. Zeit [s] = T * N.
                                
T_zs = 4;                      % Anzahl gewuenschter Zwischenschritte, in 
                                % denen ein neuer Stellwert berechnet  
                                % werden soll. Zeit [s] = T / T_zs. 
                                
u0 = zeros(1,N * T_zs + 1);     % Startwert der Stellgroesse (gleich Null). 

% OptiOptions = optimoptions('fminunc','GradObj','off','MaxIter', 2000,'Algorithm', 'trust-region','Hessian', 'off','TolFun', 0.1);
OptiOptions = optimoptions('fminunc','GradObj','off');
ODEOptions = odeset;

while((round(t_mess(end) * 100) / 100) < tspan(end))
       
    % Initialwerte
    t0_mpc = t_mess(end)           % Initialwerte sind gleich den Messwerten
    x0_mpc = x_mess(end,:);
    
    % Da wir ja einen "zeitabhaengigen" Eingang haben, muessen wir fuer
    % jeden Optimierungshorizont eine neue Zeit-Einteilung fuer den 
    % ODE-Solver generieren:

    ut = linspace(t0_mpc,t0_mpc + N * T,N * T_zs + 1);
    
    %----------------------------------------------------------------------
    % Loesung des Optimierungsproblems
    
%     % Berechnung der Loesung fuer den offenen Regelkreis
%     x_oR = LoesungRegelkreis(nmpcPendelDGS, N, T, t0, x0, u0);
%     
%     % Set control and linear bounds
%     A = [];
%     b = [];
%     Aeq = [];
%     beq = [];
%     lb = [];
%     ub = [];
%     for k=1:N
%         [Anew, bnew, Aeqnew, beqnew, lbnew, ubnew] = ...
%             linearconstraints(t0+k*T,x(k,:),u0(:,k));
%         A = blkdiag(A,Anew);
%         b = [b, bnew];
%         Aeq = blkdiag(Aeq,Aeqnew);
%         beq = [beq, beqnew];
%         lb = [lb, lbnew];
%         ub = [ub, ubnew];
%     end
%     
%     % Solve optimization problem
%     [u, ~, ~, ~] = fmincon(@(u) costfunction(runningcosts, ...
%         terminalcosts, system, N, T, t0, x0), u0, A, b, Aeq, beq, lb, ...
%         ub, @(u) nonlinearconstraints(constraints, terminalconstraints, ...
%         system, N, T, t0, x0, u), options);

    % Das bloede ist, dass unser Optimierer auch den Startwert des
    % Einganges optimieren moechte. Das ist ein Problem, weil wir den
    % ersten Eintrag des Einganges quasi schon eingestellt haben und nicht
    % mehr veraendern koennen. So geht der optimierer aber von falschen
    % Zustaenden aus und dies verringert die Qualitaet der Optimierung.
    % Deswegen muessen wir ihm sagen, die Finger vom Startwert zu lassen:
    
    Fingerweg = u0(1);

    [u_neu,~] = fminunc(@(u) costfunction(@nmpcPendelDGS, N, T_zs, T, t0_mpc, x0_mpc, l, g, ut, u, Fingerweg, ODEOptions), u0, OptiOptions);
    %----------------------------------------------------------------------
    
    % Hier werden anhand des Modells die zu erwartenen Messwerte ermittelt
    % und die daraus resultierenden Zustaende fuer die Optimierung
    % verwendet (in diskreter Zeit!).
    [t_mess, x_mess] = ode45(@nmpcPendelDGS,[t0_mpc t0_mpc + T],...
        x0_mpc, ODEOptions, l, g, ut, u_neu);
    
    % Speichern der Daten
    t_mpc = [ t_mpc; t_mess];
    x_mpc = [ x_mpc; x_mess];
    u_mpc = [ u_mpc; u_neu];
    
    % Startwert der Stellgroesse gleich der optimierten Stellgroesse
%     u0 = [u_neu((T_zs+1):end) u_neu(end) * ones(1,T_zs)];
end

u_end = [];

for k = 1 : size(u_mpc,1)
    u_end = [u_end u_mpc(k,1:T_zs)];
end

u_end(end + 1) = u_mpc(end,T_zs+1);

ut_end = linspace(tspan(1),tspan(end),tspan(end) * T_zs / T + 1);

[t_mess, x_mess] = ode45(@nmpcPendelDGS,tspan(1):T/T_zs:tspan(end),...
        x0, ODEOptions, l, g, ut_end, u_end);
    
    figure(1)
    plot(t_mess',u_end);
    
    figure(2)
    plot(t_mess',x_mess(:,3))
end

% ++++++++++++++++++++++ Differentialgleichungen +++++++++++++++++++++++++
function dx = PendelDGS(~,x,K_Pendel,K_Robo,l,g)
    u = (x(1) * K_Robo(1) + x(2) * K_Robo(2)) - (x(3) * K_Pendel(1) +...
        x(4) * K_Pendel(2)); 
    
    dx(1) = x(2);
    dx(2) = u;
    dx(3) = x(4);
    dx(4) = 1/l * cos(x(3)) * u + g/l * sin(x(3));
    
    dx = dx';
end

function dx = DiscretePendelDGS(~,x,l,g,u)
    
    dx(1) = x(2);
    dx(2) = u;
    dx(3) = x(4);
    dx(4) = 1/l * cos(x(3)) * u + g/l * sin(x(3));
    
    dx = dx';
end

function dx = nmpcPendelDGS(t, x, l, g, ut, u)

%     u = interp1(ut,u,t);
    Suche = find(ut <= t);
    u = u(Suche(end));

    dx(1) = x(2);
    dx(2) = u;
    dx(3) = x(4);
    dx(4) = u/l * cos(x(3)) + g/l * sin(x(3));
    
    dx = dx';
end

% ++++++++++++++++++++++++++ NMPC-Algorithmus +++++++++++++++++++++++++++++


function cost = costfunction(nmpcPendelDGS, N, T_zs, T, t0, x0, l, g, ...
    ut, u, Fingerweg, ODEOptions)

x = x0;
% u(1) = Fingerweg;

% Hier waehlen wir den Zeithorizont so, dass wir ein wenig ueber das
% Intervall  Messzeitpunkt hinaus berechnen, damit unser 
% Optimierer am Ende einen Effekt verspuehrt. 
% t_Ende = t0 + T * N;
t_Ende = t0 + T * (N + 1 / T_zs);

[t, x] = ode45(nmpcPendelDGS, t0:T/T_zs:t_Ende, x, ODEOptions, l, g, ut, u);

% cost = sum(sum(abs(x.y)))^2 + sum(abs(x.extdata.varargin{4}))^2;
% if (max(abs(x.y(3,:))) >= abs(90 * (2 * pi / 360)))
%     cost = 10e15;
% else
cost = sum(abs(x(:,3)));%1 * sum(abs(x(:,1))) + 1e-4 * sum(abs(x(:,2))) + 10 * sum(abs(x(:,3))) + 1e-4 * sum(abs(x(:,4))) + 0.1 * sum(abs(u));
% end

end





% function cost = costfunction(nmpcPendelDGS, N, T, t0, x0, l, g, ut, u, Fingerweg)
%     cost = 0;  
%     x = x0;
%     u(1,1) = Fingerweg;
%     t_hilf = t0;
%     
%     for k=1:N
%         x = ode45(nmpcPendelDGS, [t_hilf t_hilf + T], x, ...
%             odeset, l, g, ut(k,:), u(k,:));
% 
%         cost = cost + sum(sum(abs(x.y)))^2 + sum(abs(x.extdata.varargin{4}))^2;
% 
%         x = x.y(:,end);
%         t_hilf = t_hilf + T;
%     end
%     cost = cost + 0;
% end


% function [c,ceq] = constraints(t, x, u)
%     c   = [];
%     ceq = [];
% end
% 
% function [c,ceq] = terminalconstraints(t, x)
%     c   = [];
%     ceq = [];
% end
% 
% function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
%     A   = [];
%     b   = [];
%     Aeq = [];
%     beq = [];
%     lb  = [];
%     ub  = [];
% end



% function [c,ceq] = nonlinearconstraints(constraints, ...
%     terminalconstraints, system, ...
%     N, T, t0, x0, u)
%     x = zeros(N+1, length(x0));
%     x = LoesungOffenerRegelkreis(system, N, T, t0, x0, u);
%     c = [];
%     ceq = [];
%     for k=1:N
%         [cnew, ceqnew] = constraints(t0+k*T,x(k,:),u(:,k));
%         c = [c cnew];
%         ceq = [ceq ceqnew];
%     end
%     [cnew, ceqnew] = terminalconstraints(t0+(N+1)*T,x(N+1,:));
%     c = [c cnew];
%     ceq = [ceq ceqnew];
% end
% 