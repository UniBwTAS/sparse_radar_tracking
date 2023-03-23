function [ matrix ] = R( yaw )
%R Creates a 2D rotation matrix
%   Use: matrix = R(alpha_in_rad);

matrix = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
end

