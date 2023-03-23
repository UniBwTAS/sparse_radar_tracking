function [ angle ] = atan2i( vec )
%atan2i Implicit vectorial angle calculation
%   Use: alpha = atan2i([x;y]);

angle = atan2(vec(2), vec(1));
end

