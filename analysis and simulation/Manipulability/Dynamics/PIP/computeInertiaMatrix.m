

function M = computeInertiaMatrix(L, m1, m2, m3, m4,q5)
    L4 = 4.42; 
    L5 = 13.84; 
    L9 = 14.5; 
    L10 = 5.2; 
    L11 = 37.5; 

    L7 = L(1);
    L8 = L(2);

    L3 = sqrt(L9^2 + L10^2);
    L4 = sqrt((L11 - q5)^2 + (L4 + L5)^2);

    I1 = (1/3) * m1 * L7^2;
    I2 = (1/3) * m2 * L8^2;
    I3 = (1/3) * m3 * L3^2;
    I4 = (1/3) * m4 * L4^2;

    M = diag([I1, I2, I3, I4]);
end
