% Inverse Kinematics
function [q2, q3] = inverse_kinematics_MCP(theta1, x, theta2, q1)
    L1 = 12;
    L2 = 20;
    L3 = 45;
    L4 = 15;
    V = 47;
    H = 20;
    
    q2 = theta1;
    
    M = (H - L3 * sin(theta2)) * cos(theta1);
    N = L4 * cos(theta1);
    P = L1 - V + (L2 + L3 * cos(theta2)) * cos(theta1);
    
    q3 = acos((P - N * tan(theta2)) / (L2 + L3 * cos(theta2)));
end