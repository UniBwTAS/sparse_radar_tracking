function [z,A]=jaccsd(fun,x)
% JACCSD Jacobian through complex step differentiation
% [z J] = jaccsd(f,x)
% z = f(x)
% J = f'(x)
%
z=fun(x);
n=numel(x);
m=numel(z);
A=zeros(m,n);
h=n*eps;
for k=1:n
    x1=complex(x);
    x1(k)=x1(k)+h*1i;
    A(:,k)=imag(fun(x1))/h;
end

%% Notes
% Source: https://github.com/nesl/PenguinTracking/blob/master/src/matlab/utilities/jaccsd.m
% Presumable origin: https://de.mathworks.com/matlabcentral/fileexchange/18176-complex-step-jacobian