% Function to compute global manipulability
function global_manipulability = computeGlobalManipulability(L, Theta, Q5, W)
    global W_v;    
    global_manipulability = 0;
    num_samples = 0;
    for i = 1:size(Theta, 1)
        for j = 1:size(Theta, 2)
            theta3 = Theta(i, j);
            q5 = Q5(i, j);
            q6 = inverse(L, q5, theta3);

            J = jacobian(L, q5, q6);

            JW_inv = inv(J)' * W';
            JW_inv1 = W * inv(J);

            local_manipulability = sqrt(det(inv(JW_inv * JW_inv1)));
            W_v(i, j) = local_manipulability;

            global_manipulability = global_manipulability + local_manipulability;
            num_samples = num_samples + 1;
        end
    end
    global_manipulability = global_manipulability / num_samples;
end