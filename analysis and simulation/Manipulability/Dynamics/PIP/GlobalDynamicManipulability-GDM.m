function global_manipulability = computeGlobalDynamicManipulability(L, Theta, Q5, W)
    global W_a; 
    global_manipulability = 0;
    num_samples = 0;

    m1 = 1; % Mass of link 1 (L7)
    m2 = 1; % Mass of link 2 (L8)
    m3 = 1; % Mass of link 3 (sqrt(L9^2 + L10^2))
    m4 = 1; % Mass of link 4 (sqrt((L11 - q5)^2 + (L4 + L5)^2))

    for i = 1:size(Theta, 1)
        for j = 1:size(Theta, 2)
            theta3 = Theta(i, j);
            q5 = Q5(i, j);

            q6 = inverse(L, q5, theta3);

            J = jacobian(L, q5, q6);

            M = computeInertiaMatrix(L, m1, m2, m3, m4, theta3, q5);

            D = J * inv(M) * J';
            local_manipulability = sqrt(det(D));
            

            W_v(i, j) = local_manipulability;

            global_manipulability = global_manipulability + local_manipulability;
            num_samples = num_samples + 1;
        end
    end

    global_manipulability = global_manipulability / num_samples;
end


