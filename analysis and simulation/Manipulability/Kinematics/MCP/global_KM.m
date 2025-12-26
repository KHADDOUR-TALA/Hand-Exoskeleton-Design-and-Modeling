 syms alpha1 theta
    L2=20;L4=4;L5=11;L6=36;H=20;V=47;dirac1=0;dirac2=0;dirac3=0; 
    q2_max = 20*pi/180;
    q3_max = 60*pi/180;

    L1_range = linspace(10,25,1);
    L3_range = linspace(35,55,1);
    [L_1,L_3] = meshgrid(L1_range, L3_range);

    W_v = sym(zeros(size(L_1)));
     
    W = diag([1/q2_max, 1/q3_max]);
    
    for i = 1:length(L1_range)
        for j = 1:length(L3_range)
            L1 = L_1(j, i);
            L3 = L_3(j, i);
            J = jacobian2T(L1,L3);
            JW_inv = inv(J)' *W';
            JW_inv1 = W*inv(J);
            manipulability = simplify(sqrt(det(inv(JW_inv * JW_inv1))))
            m1=int(manipulability,alpha1,-10, 10);        
        end
    end
    
    figure;
    surf( L_1,L_3,W_v);
    xlabel('\theta_1');
    ylabel('\theta_2');
    zlabel('Kinematic Manipulability Measure W_v');
    title('Kinematic Manipulability Measure');
    colorbar;

