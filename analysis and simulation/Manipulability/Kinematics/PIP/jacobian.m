function J = jacobian(L, q5, q6)
    delta = 1e-6; % Small perturbation
    theta3_plus = computeTheta3(L, q5, q6 + delta);
    theta3_minus = computeTheta3(L, q5, q6 - delta);
    J = (theta3_plus - theta3_minus) / (2 * delta);
    J;
end