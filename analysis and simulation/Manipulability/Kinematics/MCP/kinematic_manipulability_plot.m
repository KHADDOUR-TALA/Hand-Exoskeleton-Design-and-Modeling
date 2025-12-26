L=[12,20,45,15];
q2_max = 20*pi/180;
q3_max = 60*pi/180;


alpha_range = linspace(-20*pi/180, 20*pi/180,100);
theta_range = linspace(-30*pi/180, 90*pi/180, 100);
[Alpha1, Theta] = meshgrid(alpha_range, theta_range);

W_v = zeros(size(Alpha1));

W = diag([1/q2_max, 1/q3_max]);

for i = 1:length(alpha_range)
    for j = 1:length(theta_range)
        alpha = Alpha1(j, i);
        theta = Theta(j, i);

        [q2, q3] = inverse_T(L,alpha, theta) ; 

        J = jacobian_T(L,q2, q3);

        JW_inv = inv(J)' *W';
        JW_inv1 = W*inv(J);
        manipulability = sqrt(det(inv(JW_inv * JW_inv1)));

        W_v(j, i) = manipulability;

    end
end
figure;
surf( Theta,Alpha1, W_v);
xlabel('\theta');
ylabel('\alpha');
zlabel('Kinematic Manipulability Measure W_v');
title('Kinematic Manipulability Measure');
colorbar;

