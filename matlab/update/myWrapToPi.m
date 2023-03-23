function [ out ] = myWrapToPi( in )
%myWrapToPi "MATLAB Coder"-compatible "wrapToPi()" variant
%   Detailed explanation does not go here

out = in - 2*pi*floor( (in+pi)/(2*pi) );
end

