function  [alpha, beta ,gamma] = get_orientation(matrix)
    % Computes de Euler Angles in Z-Y-Z convenction
    alpha = 0;                                                          % Z
    beta = atan2(sqrt(matrix(1,3)^2 + matrix(2,3)^2), matrix(3,3));     % Y
    gamma = 0;                                                          % Z
    

    if round(beta,3) == round(pi/2,3)
        gamma = atan2(matrix(2,1), matrix(1,1));

    elseif round(beta,3) == round(-pi/2,3)
        gamma = -atan2(matrix(2,1), matrix(1,1));

    elseif round(beta,3) == 0
        gamma = 0;
        alpha = cos(beta)*atan2(matrix(2,1), matrix(1,1)); % cos(beta) -> ±1
    else
        alpha = atan2(matrix(2,3)/sin(beta), matrix(1,3)/sin(beta));
        gamma = atan2(matrix(3,2)/sin(beta), -matrix(3,1)/sin(beta));
    end

    matrix_test = rotz(alpha*180/pi)*roty(beta*180/pi)*rotz(gamma*180/pi);

      
end