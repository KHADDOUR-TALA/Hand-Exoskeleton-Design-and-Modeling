%%%%workspace of the MCP close-loop chain%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i=1;
q2_max=20;
q2_min=-20;
q3_max=60;
q3_min=-20;
s=0.5;
theta1=zeros((q2_max-q2_min+1)*(q3_max-q3_min+1)/s);
theta2=zeros((q2_max-q2_min+1)*(q3_max-q3_min+1)/s);
for q2=q2_min:s:q2_max;
    for q3=3:s:q3_max;
        [ theta1(i),x,theta2(i),q1] = forward_kinematics_MCP(q2*pi/180,q3*pi/180);
        i=i+1;
    end
end
theta1=theta1*180/pi;
theta2=theta2*180/pi;
figure 
plot(theta1,theta2,'b.')
xlabel('\theta (radians)')
ylabel('\theta (radians)')
