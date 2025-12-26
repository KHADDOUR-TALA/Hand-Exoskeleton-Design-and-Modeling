function [ bounded_theta ] = bound_angle( theta,min,max )

bounded_theta=theta;
if (theta<min)
    bounded_theta=min;
end
if(theta>max)
    bounded_theta=max;
end

end

