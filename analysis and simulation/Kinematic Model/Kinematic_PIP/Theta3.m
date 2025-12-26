function theta3 = computeTheta3(L, q5, q6)
    L4 = 4.42; 
    L5 = 13.84; 
    L9 = 14.5; 
    L10 = 5.2; 
    L11 = 37.5; 
    L7 = L(1);
    L8 = L(2);

    E = (L4 + L5)^2 + (L11 - q5)^2;

    theta3 = acos((E + L10^2 + L9^2 - L7^2 - L8^2 - 2 * L7 * L8 * cos(q6)) / (2 * sqrt(E * (L9^2 + L10^2)))) + atan((L4 + L5) / (L11 - q5)) + atan(L9 / L10) - pi;
end