

function skyjumper
clear all
clc
% optimal control Skyjumper

%% Parameter
%Kostenfunktion
%J=x(T);
%Kr�fte
% F_D=0.5*rho*v^2*A*cd;%alpha
% F_L=0.5*rho*v^2*A*cl;%alpha
%initial
%t=0;;v_0=1;theta=theta_0;

%ODE2
% % % % dpvdt=pv*2*c_d*sigma*v+psi;
% % % % dp_theta_dt=ptheta*sin(theta)/v+ksi;
%%
%inital
%t=T;pv=0;ptheta=0;
% %% lambda=bcfun
% lambda1=-(cos(beta))*v_0/g*cos(theta)-(-cos(beta)* acosd(theta))*v_0^2/g*sin(theta)...
%     +2*pv*c_d*sigma*v -p_theta*(c_l*sigma+cos(theta)/v^2);%dpvdt
% 
% lambda2=(cos(beta))*v_0/g*sin(theta)-(-cos(beta)* acosd(theta))*v_0^2/g*cos(theta)...
%     + pv*cos(theta) -p_theta*(sin(theta)/v);%dpthetadt
% 
% %% l�ser
% 
% sol = bvp4c(odefun,bcfun,solinit,options)
% solinit = bvpinit(x, yinit, params) %gesch�tzte Anfangsbedingung
% 
% 
% %% optimal alpha
% alpha_star=(b_1*pv*v-b_2*ptheta)/(2*a_2*ptheta-2*a_1*pv);

%plotten

%% Fit data to tracked cL and cD
% data from tracking
cd = [0.119 0.219 0.320 0.462 0.657];
cl = [0.117 0.279 0.390 0.472 0.346];
alpha_g = [10 20 30 40 60];
 
% Least square fitting
% cd_coeff(1) = a
% cd_coeff(2) = b
% cd_coeff(3) = d
% cl_coeff(1) = a'
% cl_coeff(2) = b'
% cl_coeff(3) = d'
p.cd_coeff = polyfit(alpha_g,cd,2);
p.cl_coeff = polyfit(alpha_g,cl,2);

rho=1.22;           %kg m^-3
A_durch_m=0.01;      %m^2*kg^-1
%beta=pi/4;

p.v_0=35;
p.g=9.81;
p.sigma=A_durch_m*rho*p.v_0^2/(2*p.g);
p.tend=5;%/(p.v_0/p.g);

%% Optimierung
anzahl_alpha = 20;
%alpha_min(1:anzahl_alpha)  = pi/4;
alpha_min(1:anzahl_alpha)  = 0.1;
alpha_max(1:anzahl_alpha)  = pi/4;
%alpha0(1:anzahl_alpha)     = pi/4/3/2;
alpha0= [20/180*pi:(pi/4-pi/9)/(anzahl_alpha-1):pi/4]

%alpha0=[1/5*pi,1/5*pi,1/5*pi,1/5*pi,1/5*pi,1/5*pi,1/5*pi,1/5*pi,1/5*pi,1/5*pi];

%alpha_stern=fminunc(@zielfunktion,alpha0);
%%%%%%%alpha_stern = fmincon(@(alpha)zielfunktion(alpha,p),alpha0,[],[],[],[],alpha_min,alpha_max)
%alpha_stern = fmincon(@zielfunktion,alpha0,A,B);


v_0=p.v_0/p.v_0;
%v_0=p.v_0;
theta_0=-0.0903;    %rad -5.17�
%alpha_0=pi/2;       % 90�
x_0=0;
y_0=0;
x0=[v_0,theta_0,x_0,y_0];
% 
% for i=1:5
%     alpha0(1:anzahl_alpha) = pi/4/5*i;


tend=p.tend/(p.v_0/p.g);
%T=[1:1:tend/2];
T=[0:0.2/(p.v_0/p.g):tend];
%T=[0:0.2:tend]
options=odeset('MaxStep',0.001);
[t,X]=ode15s(@(t,X)ode(t,X,alpha_max,p),T,x0)
% % % 
% % % 
% % % solinit = bvpinit(linspace(0,4,5),[1 0]);
% % % sol = bvp4c(@twoode,@twobc,solinit);

figure(1)
%subplot 21
%plot(t,alpha);
%xlabel('Zeit t');ylabel('alpha');
%subplot 22
plot(X(:,3),X(:,4),'g');
hold on
x_slope=[0 X(end,3)];
y_slope=-0.0008*x_slope-5;
plot(x_slope,y_slope,'g');
xlabel('x');ylabel('y');
% end

end 

function x = zielfunktion(alpha,p )


v_0=p.v_0/p.v_0;
theta_0=-0.0903;    %rad -5.17�
%alpha_0=pi/2;       % 90�
x_0=0;
y_0=0;
x0=[v_0,theta_0,x_0,y_0];

tend=p.tend/(p.v_0/p.g);
%T=[1:1:tend/2];
T=[0:0.2/(p.v_0/p.g):tend];

options=odeset('MaxStep',0.1);

   [t,X]=ode15s(@(t,X)ode(t,X,alpha,p),T,x0);

% strafterm = 0;
% for i=1:p.anzahl_alpha 
%     if p.alpha(i)>pi/4 || p.alpha(i)<-pi/4
%         strafterm=strafterm+(p.alpha(i)-pi/4)^4
%     end
% end
%   x = -X(end,3);
   x = X(end,3)/X(end,4)
%   x = x+strafterm;
end
% % function dXdt=ode_ohne_substitution(t,X,alpha,p)
% %     anzahl_alpha = length(alpha);
% %     for i=1:anzahl_alpha
% %     if t>=p.tend/anzahl_alpha*(i-1) && t<=p.tend/anzahl_alpha*(i)
% %         alpha=alpha(i);
% %     end
% %     end
% %     v=X(1); theta=X(2);
% %     x=X(3); y=X(4);
% %     
% %     dvdt=-(p.cd_coeff(1)*alpha^2+p.cd_coeff(2)*alpha+p.cd_coeff(3))/p.m
% % end
function dXdt=ode(t,X,alpha,p)

anzahl_alpha = length(alpha);
for i=1:anzahl_alpha
if t>=p.tend/(p.v_0/p.g)/anzahl_alpha*(i-1) && t<=p.tend/(p.v_0/p.g)/anzahl_alpha*(i)
    alpha=alpha(i);
end
end
v=X(1);
theta=X(2);
x=X(3);
y=X(4);

%% ODE1=odefun
dvdt=-(p.cd_coeff(1)*alpha^2+p.cd_coeff(2)*alpha+p.cd_coeff(3))*p.sigma*v^2-sin(theta);     %(-F_D-m*g*sin(theta))/m;
dthetadt=(p.cl_coeff(1)*alpha^2+p.cl_coeff(2)*alpha+p.cl_coeff(3))*p.sigma*v-cos(theta)/v;  %(-F_L-m*g*cos(theta))/(m*v);
dxdt=v*p.v_0^2 /p.g *cos(theta)*1000;                               %v*cos(theta);
dydt=v*p.v_0^2/p.g*sin(theta);                      % v*sin(theta);

dXdt=[dvdt, dthetadt, dxdt, dydt]';
end
