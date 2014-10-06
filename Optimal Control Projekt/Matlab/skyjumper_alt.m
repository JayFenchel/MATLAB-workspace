function skyjumper
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

% 
% 
% function dxdt=odefun(t,x0,p)
% %% ODE1=odefun
% dvdt=-cd*sigma*v^2-sin(theta);     %(-F_D-m*g*sin(theta))/m;
% dthetadt=cl*sigma*v-cos(theta)/v;  %(-F_L-m*g*cos(theta))/(m*v);
% dxdt=v*v_0^2 /g *cos(theta);                               %v*cos(theta);
% dydt=v*v_0^2/g*sin(theta);     
% 
% end
% 
% 
% function dlambdadt=bcfun(t,x0,p)
% 
% 
% end

A=[];
b=[];


alpha0=[1/4*pi];
alpha_stern=fminunc(@zielfunktion,alpha0);
alpha_stern

% % figure(1)
% % subplot 21
% % plot(t,alpha);xlabel='Zeit t';ylabel='alpha';
% % subplot 22
% % plot(y,x);xlabel='x-Stelle';ylabel='y-Stelle';

end 

function x = zielfunktion(alpha )
global p

p.alpha=alpha;

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

p.v_0=20;
p.g=9.81;
p.sigma=A_durch_m*rho*p.v_0^2/(2*p.g);

v_0=20;
theta_0=-0.0903;    %rad -5.17�
%alpha_0=pi/2;       % 90�
x_0=0;
y_0=0;
x0=[v_0,theta_0,x_0,y_0];

tend=20;
T=[1:0.1:tend];

   [X,t]=ode15s(@ode,T,x0);
   x=-X(3,end);
    
end

function dXdt=ode(t,X)
global p

v=X(1);
theta=X(2);
x=X(3);
y=X(4);

%% ODE1=odefun
dvdt=-(p.cd_coeff(1)*p.alpha^2+p.cd_coeff(2)*p.alpha+p.cd_coeff(3))*p.sigma*v^2-sin(theta);     %(-F_D-m*g*sin(theta))/m;
dthetadt=(p.cl_coeff(1)*p.alpha^2+p.cl_coeff(2)*p.alpha+p.cl_coeff(3))*p.sigma*v-cos(theta)/v;  %(-F_L-m*g*cos(theta))/(m*v);
dxdt=v*p.v_0^2 /p.g *cos(theta);                               %v*cos(theta);
dydt=v*p.v_0^2/p.g*sin(theta);                      % v*sin(theta);

dXdt=[dvdt, dthetadt, dxdt, dydt]';
end
