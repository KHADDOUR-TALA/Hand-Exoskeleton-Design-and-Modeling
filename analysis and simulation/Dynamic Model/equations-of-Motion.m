function [M, C, G] = equationsOfMotion(q, dq, ddq, EOM)
    % Number of joints
    n = length(q);
    
    % Initialize matrices
    M = sym(zeros(n));
    C = sym(zeros(n, n));
    G = sym(zeros(n, 1));
    
    % Extract the elements
    for i = 1:n
        for j = 1:n
            M(i, j) = functionalDerivative(EOM(i), ddq(j));
        end
        G(i) = -functionalDerivative(EOM(i), g);
    end
    
    % Compute Coriolis and centrifugal matrix
    for i = 1:n
        for j = 1:n
            C(i, j) = 0;
            for k = 1:n
                C(i, j) = C(i, j) + 0.5 * (functionalDerivative(M(i, j), q(k)) + functionalDerivative(M(i, k), q(j)) - functionalDerivative(M(j, k), q(i))) * dq(k);
            end
        end
    end
end
