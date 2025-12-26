% Define the ranges for L1 and L4 for the surface plot
L1_range = linspace(10, 25, 20);
L4_range = linspace(10, 19, 20);
L2_range = linspace(15, 25, 20);
L3_range = linspace(35, 55, 20);
[LL1, LL4] = meshgrid(L1_range, L4_range);

global_manipulability_surface = zeros(size(LL1));
CPI_surface = zeros(size(LL1));

% Calculate global manipulability and CPI for each combination of L1 and L4


% Calculate m and n for CPI calculation

%%n = mean(W_v, 'all');

% Calculate CPI surface
% for i = 1:length(L1_range)
%     for j = 1:length(L4_range)
%         global_manipulability = global_manipulability_surface(j, i);
%         W_ga = W_v(j, i);
%         CPI_surface(j, i) = global_manipulability / m + W_ga / n;
%     end
% end

% Plot the global manipulability surface

% Define initial guesses for link lengths
L0 = [12,20,45,15]; % Initial guesses for link lengths L1, L2, L3
q2_max = 20*pi/180;
    q3_max = 60*pi/180;
tq2_max = 0.4;
    tq3_max = 0.3;

% Define the ranges for joint angles
alpha_range = linspace(-20*pi/180, 20*pi/180,5);
theta_range = linspace(-30*pi/180, 90*pi/180,5);
[Alpha1, Theta] = meshgrid(alpha_range, theta_range);
W_v = zeros(size(Alpha1));
 W_a = zeros(size(Alpha1));
 W_t = zeros(size(Alpha1));
    % Weighting matrix
W = diag([1/q2_max, 1/q3_max]);
Wd = diag([1/tq2_max, 1/tq3_max]);

% Set optimization options
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

% Objective function to maximize global manipulability
objective = @(L) -compute(L,Alpha1, Theta,W,Wd,L1_range,L4_range,L2_range,L3_range);

% Optimization constraints (if any, e.g., positive lengths)
constraints = [];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [10,15,35,10]; % Lower bounds for L1, L2, L3
ub = [25,25,55,19]; % Upper bounds for L1, L2, L3

% Perform the optimization
[L_opt, fval] = fmincon(objective, L0, A, b, Aeq, beq, lb, ub, constraints, options);

% Display optimized link lengths
disp(['Optimized Link Lengths: L1 = ', num2str(L_opt(1)), ', L2 = ', num2str(L_opt(2)),  ', L3 = ',num2str(L_opt(3)), ', L4 = ', num2str(L_opt(4))]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function CPI=compute(L,Alpha1, Theta,W,Wd,L1_range,L4_range,L2_range,L3_range)
[global_manipulabilityk,global_manipulabilityt] = computeGlobalManipulability(L,Alpha1, Theta,W,Wd);
[m,n]=computeCPI(L0,L1_range,L4_range,L2_range,L3_range,Alpha1, Theta, W,Wd);
CPI= global_manipulabilityk / m +  global_manipulabilityt / n;
end
%%%%%%%%%%%
function [m,n]=computeCPI(L0,L1_range,L4_range,L2_range,L3_range,Alpha1, Theta, W,Wd)
for i = 1:length(L1_range)
    for j = 1:length(L4_range)
        L = [L1_range(i), L0(2), L0(3), L4_range(j)]; % Fix L2 and L3
        
        % Compute global manipulability
       [ global_manipulability_surfacev(j, i), global_manipulability_surfacet(j, i)] = computeGlobalManipulability(L, Alpha1, Theta, W);
    end
end


%  for i = 1:length(L1_range)
%     for j = 1:length(L4_range)
%         global_manipulabilityk = global_manipulability_surface_k(j, i);
%         global_manipulabilityt = global_manipulability_surface_t(j, i);;
%         CPI_surface(j, i) = global_manipulabilityk / m +  global_manipulabilityt / n;
%     end
% end
 end
% Function to compute global manipulability
function [ global_manipulability_a, global_manipulability_t] = computeGlobalManipulability(L,Alpha1, Theta,W,Wd)

    global_manipulability_k = 0;
     global_manipulability_a = 0;
      global_manipulability_t = 0;
    num_samples = 0;
    n=0;
    % Loop over all configurations to compute local manipulability
    for i = 1:length(Alpha1)
        for j = 1:length(Theta)
            alpha11 = Alpha1(j, i);
            theta = Theta(j, i);
            
            % Compute the Jacobian at this configuration
           [q2, q3] = inverse_T(L,alpha11, theta);  % Note: x and q1 are not used in inverse kinematics here

            % Compute the Jacobian matrix for the current q2 and q3
            J = jacobian_T(L,q2, q3);
  
            % Compute the manipulability measure
            JW_inv = inv(J)' *W';
            JW_inv1 = W*inv(J);
            
            % Compute the local kinematic manipulability measure
            local_manipulability_k =  sqrt(det(inv(JW_inv * JW_inv1)));
             W_v(j, i) = local_manipulability_k;
            % Accumulate for global manipulability
            global_manipulability_k = global_manipulability_k + local_manipulability_k;
            num_samples = num_samples + 1;
            [M,Me]=inertia_matrix_exo(L,q2,q3);
            % Compute the manipulability measure
           B1=M*inv(J)+J'*Me;
              B2=M*inv(J)*inv(Me)+J';
            % Compute the manipulability measure
            
            manipulability1 = sqrt(det(inv(B1'*Wd'*Wd*B1)));
            manipulability2 = sqrt(det(inv(B2'*Wd'*Wd*B2)));
            
            % Store the manipulability measure
            W_a(j, i) = manipulability1;
             W_t(j, i) = manipulability2;
            
            % Compute the local kinematic manipulability measure
           
            % Accumulate for global manipulability
            global_manipulability_a = global_manipulability_a + manipulability1;
             global_manipulability_t = global_manipulability_t + manipulability2;
            num_samples = num_samples + 1;
        end
    end
    
     
    % Average global manipulability
    global_manipulability_k = global_manipulability_k / num_samples;
    global_manipulability_a= global_manipulability_a/ num_samples;
    global_manipulability_t= global_manipulability_t/ num_samples;
CPI=(global_manipulability_k+0.5*global_manipulability_a+0.5*global_manipulability_t)/3
   % W_gv(j, i) = global_manipulability;
end

% Function to compute the Jacobian matrix
