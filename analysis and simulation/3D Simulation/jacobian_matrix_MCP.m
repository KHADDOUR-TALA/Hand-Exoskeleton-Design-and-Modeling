% Jacobian Matrix
function J = jacobian_matrix_MCP(q2, q3)
    syms q2_sym q3_sym real
    L1 = 12;
    L2 = 20;
    L3 = 45;
    L4 = 15;
    V = 47;
    H = 20;
    
    % Forward Kinematics Equations
    theta1 = q2_sym;
    M = (H - L3 * sin(q3_sym)) * cos(q2_sym);
    N = L4 * cos(q2_sym);
    P = L1 - V + (L2 + L3 * cos(q3_sym)) * cos(q2_sym);
    theta2 = 2 * atan((M + sqrt(M^2 - N^2 + P^2)) / (N + P));
    x = (H - L4 * sin(theta2 + pi/2) - L3 * sin(q3_sym)) / cos(theta2 + pi/2);
    q1 = (L1 - V) * tan(q2_sym);
    
    % State vector
    X = [theta1; x; theta2; q1];
    % Control vector
    phi = [q2_sym; q3_sym];
    J = jacobian(X, phi);
    J = double(subs(J, {q2_sym, q3_sym}, {q2, q3}));
end