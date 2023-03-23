function [z,A]=jaccsdg(fun,x,par)
% JACCSDG Jacobian through complex step differentiation with custom state-independent parameters
% [z J] = jaccsd(f,x, par)
% z = f(x, par)
% J = f'(x, par)

z=fun(x,par);
n=numel(x);
m=numel(z);
A=zeros(m,n);
h=n*eps;
for k=1:n
    x1=complex(x);
    x1(k)=x1(k)+h*1i;
    A(:,k)=imag(fun(x1,par))/h;
end

%% Notes
% - Adaption of "jaccsd.m" supporting custom (fixed) arguments for integration in Lambda functions
% - Use with attention (e.g. transposes like "[A]'" in "fun" will affect the complex number)