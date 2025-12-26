L0 = [25, 23]; % Initial guesses for link lengths L7, L8
q6_max = 60 * pi / 180;

theta_range = linspace(-30 * pi / 180, 90 * pi / 180, 100); 
q5_range = linspace(5.5, 30, 100); 
[Theta, Q5] = meshgrid(theta_range, q5_range);

global W_v; 
W_v = zeros(size(Theta)); 

W = diag([1 / q6_max]);

options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

objective = @(L) -computeGlobalDynamicManipulability(L, Theta, Q5, W);

constraints = [];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [10, 15]; 
ub = [25, 30]; 

[L_opt, fval] = fmincon(objective, L0, A, b, Aeq, beq, lb, ub, constraints, options);

disp(['Optimized Link Lengths: L7 = ', num2str(L_opt(1)), ', L8 = ', num2str(L_opt(2))]);

figure;
surf(Q5, Theta, W_v);
xlabel('q5');
ylabel('\theta_3');
zlabel('Dynamic Manipulability Measure W_v');
title('Dynamic Manipulability Measure');
colorbar;
