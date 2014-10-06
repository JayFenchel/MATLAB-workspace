
%% Fit data to tracked cL and cD
% data from tracking
cd = [0.119 0.219 0.320 0.462 0.657];
cl = [0.117 0.279 0.390 0.472 0.346];
alpha = [10 20 30 40 60];
 
% Least square fitting
% cd_coeff(1) = a
% cd_coeff(2) = b
% cd_coeff(3) = d
% cl_coeff(1) = a'
% cl_coeff(2) = b'
% cl_coeff(3) = d'
cd_coeff = polyfit(alpha,cd,2)
cl_coeff = polyfit(alpha,cl,2);
 
% Solution Plot like in paper
cd_sol = polyval(cd_coeff,7:1:72);
cl_sol = polyval(cl_coeff,7:1:72);
cd_dot = polyval(cd_coeff,10:10:70);
cl_dot = polyval(cl_coeff,10:10:70);
 
figure(1)
plot(cd_sol,cl_sol, '-k', cd_dot, cl_dot, 'or'), xlabel('c_d'), ylabel('c_l'), grid
axis([0 0.8 0 0.601])
