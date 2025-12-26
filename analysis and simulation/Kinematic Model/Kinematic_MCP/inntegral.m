% Define the function to be integrateds
syms th
f =cos(th)*11000+sin(th);
a=zeros(1,1)
% Define the limits of integration
x_min = 0;
x_max = 2;
y_min = 0;
y_max = 1;

% Perform the double integration
result = double(int(f,th, x_min, x_max));

% Display the result
disp('The result of the double integration is:');
disp(result);