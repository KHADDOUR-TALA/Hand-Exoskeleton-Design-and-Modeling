% Define initial guesses for link lengths
L0 = [11,15,35,10]; % Initial guesses for link lengths L1, L2, L3
q2_max = 20*pi/180;
    q3_max = 60*pi/180;

% Define the ranges for joint angles
alpha_range = linspace(-20*pi/180, 20*pi/180,20);
theta_range = linspace(-30*pi/180, 90*pi/180,20);
[Alpha1, Theta] = meshgrid(alpha_range, theta_range);
W_v = zeros(size(Alpha1));
 
    % Weighting matrix
W = diag([1/q2_max, 1/q3_max]);
% Set optimization options
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

% Objective function to maximize global manipulability
objective = @(L) -computeGlobalManipulability(L,Alpha1,Theta,W);

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
disp(['Optimized Link Lengths: L1 = ', num2str(L_opt(1)), ', L2 = ', num2str(L_opt(2))]);
 
% Function to compute global manipulability
function global_manipulability = computeGlobalManipulability(L,Alpha1, Theta,W)
    global_manipulability_a = 0;global_manipulability_t = 0;
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
   [[M,Me]=inertia_matrix_exo(L,q2,q3);
            % Compute the manipulability measure
           B1=M*inv(J)+J'*Me;
              B2=M*inv(J)*inv(Me)+J';
            % Compute the manipulability measure
            
            manipulability1 = sqrt(det(inv(B1'*W'*W*B1)));
            manipulability2 = sqrt(det(inv(B2'*W'*W*B2)));
            
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
    global_manipulability = global_manipulability_a / num_samples;
    
  
end

