function J = jacobian(L, q5, q6)
    % Numerical differentiation to find the Jacobian
    delta = 1e-6; 
    theta3_plus = computeTheta3(L, q5, q6 + delta);
    theta3_minus = computeTheta3(L, q5, q6 - delta);
    J = (theta3_plus - theta3_minus) / (2 * delta);
    J;
end