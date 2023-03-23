function [value] = computeIntegralSpread(covariance3x3, pointStart, pointEnd, point, mean, total_probability)
%COMPUTEINTEGRALMEAN Spread of an integrated normal distribution along a 3D line
%   covariance3x3:      3D covariance matrix (2D positional, 1D kinematical)
%   pointStart:         3D point of on end point of the line
%   pointEnd:           3D point of the other end point of the line
%   point:              requested 3D point, for which the probabilistic mean origin has been computed
%   mean:               result from "computeIntegralMean()" executed with the upper parameters (to optimize runtime)
%   total_probability:  result from "computeIntegralProb()" executed with the upper parameters (to optimize runtime)
%   Returns the spread of means for "mean" (in [0;1]^2).

% Helper functions
pyth = @(vec) sqrt(vec(1)^2 + vec(2)^2);

% Coordinate rotation and scaling to simplify the integral
use_length = pyth(pointStart(1:2) - pointEnd(1:2));
use_angle = atan2(pointEnd(2) - pointStart(2), pointEnd(1) - pointStart(1));
point2 = R(-use_angle) * (point(1:2) - pointStart(1:2));
use_noise = R(-use_angle) * covariance3x3(1:2,1:2) * R(-use_angle)';
use_point = [point2; point(3) - pointStart(3)];
use_dopplerLength = pointEnd(3) - pointStart(3);
a_d = use_dopplerLength/use_length;

% Workaround for MATLAB Coder compatibility
pi_ = pi;
erf_rfun = @(x) erf(real(x));
sqrt_cfun = @(x) sqrt(complex(x));

% Integral
value = real(sqrt(2.0).*1.0./pi_.^(3.0./2.0).*sqrt(covariance3x3(3,3)).*exp(((-use_noise(2,1).^2.*use_point(3).^2+use_noise(1,1).*use_noise(2,2).*use_point(3).^2+use_noise(2,2).*covariance3x3(3,3).*use_point(1).^2+use_noise(1,1).*covariance3x3(3,3).*use_point(2).^2-use_noise(2,1).*covariance3x3(3,3).*use_point(1).*use_point(2).*2.0).*(-1.0./2.0))./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))).*sqrt_cfun(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*1.0./(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)).^2.*(a_d.*use_noise(2,1).^2.*use_point(3)-a_d.^2.*use_noise(2,1).^2.*mean(1).*2.0+use_noise(2,2).*covariance3x3(3,3).*mean(1).*2.0-use_noise(2,2).*covariance3x3(3,3).*use_point(1)+use_noise(2,1).*covariance3x3(3,3).*use_point(2)-a_d.*use_noise(1,1).*use_noise(2,2).*use_point(3)+a_d.^2.*use_noise(1,1).*use_noise(2,2).*mean(1).*2.0).*(-1.0./4.0)+(sqrt(2.0).*1.0./pi_.^(3.0./2.0).*sqrt(covariance3x3(3,3)).*exp(((-use_noise(2,1).^2.*use_point(3).^2+use_length.^2.*use_noise(2,2).*covariance3x3(3,3)+use_noise(1,1).*use_noise(2,2).*use_point(3).^2+use_noise(2,2).*covariance3x3(3,3).*use_point(1).^2+use_noise(1,1).*covariance3x3(3,3).*use_point(2).^2-a_d.^2.*use_length.^2.*use_noise(2,1).^2-use_length.*use_noise(2,2).*covariance3x3(3,3).*use_point(1).*2.0+use_length.*use_noise(2,1).*covariance3x3(3,3).*use_point(2).*2.0-use_noise(2,1).*covariance3x3(3,3).*use_point(1).*use_point(2).*2.0+a_d.*use_length.*use_noise(2,1).^2.*use_point(3).*2.0+a_d.^2.*use_length.^2.*use_noise(1,1).*use_noise(2,2)-a_d.*use_length.*use_noise(1,1).*use_noise(2,2).*use_point(3).*2.0).*(-1.0./2.0))./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))).*sqrt_cfun(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*1.0./(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)).^2.*(a_d.*use_noise(2,1).^2.*use_point(3)-a_d.^2.*use_noise(2,1).^2.*mean(1).*2.0+use_noise(2,2).*covariance3x3(3,3).*mean(1).*2.0-use_noise(2,2).*covariance3x3(3,3).*use_point(1)+use_noise(2,1).*covariance3x3(3,3).*use_point(2)-a_d.*use_noise(1,1).*use_noise(2,2).*use_point(3)+a_d.^2.*use_noise(1,1).*use_noise(2,2).*mean(1).*2.0))./4.0-(sqrt(2.0).*use_length.*1.0./pi_.^(3.0./2.0).*sqrt(covariance3x3(3,3)).*exp(((-use_noise(2,1).^2.*use_point(3).^2+use_length.^2.*use_noise(2,2).*covariance3x3(3,3)+use_noise(1,1).*use_noise(2,2).*use_point(3).^2+use_noise(2,2).*covariance3x3(3,3).*use_point(1).^2+use_noise(1,1).*covariance3x3(3,3).*use_point(2).^2-a_d.^2.*use_length.^2.*use_noise(2,1).^2-use_length.*use_noise(2,2).*covariance3x3(3,3).*use_point(1).*2.0+use_length.*use_noise(2,1).*covariance3x3(3,3).*use_point(2).*2.0-use_noise(2,1).*covariance3x3(3,3).*use_point(1).*use_point(2).*2.0+a_d.*use_length.*use_noise(2,1).^2.*use_point(3).*2.0+a_d.^2.*use_length.^2.*use_noise(1,1).*use_noise(2,2)-a_d.*use_length.*use_noise(1,1).*use_noise(2,2).*use_point(3).*2.0).*(-1.0./2.0))./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))).*sqrt_cfun(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))./(use_noise(2,2).*covariance3x3(3,3).*4.0-a_d.^2.*use_noise(2,1).^2.*4.0+a_d.^2.*use_noise(1,1).*use_noise(2,2).*4.0)+1.0./pi_.^(3.0./2.0).*1.0./sqrt(covariance3x3(3,3)).*sqrt(pi).*erf_rfun((sqrt(2.0).*1.0./sqrt_cfun(-(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2))./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))).*(a_d.*use_noise(2,1).^2.*use_point(3)-a_d.^2.*use_length.*use_noise(2,1).^2+use_length.*use_noise(2,2).*covariance3x3(3,3)-use_noise(2,2).*covariance3x3(3,3).*use_point(1)+use_noise(2,1).*covariance3x3(3,3).*use_point(2)-a_d.*use_noise(1,1).*use_noise(2,2).*use_point(3)+a_d.^2.*use_length.*use_noise(1,1).*use_noise(2,2)).*5.0e-1i)./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))).*exp(((use_noise(2,2).*use_point(3).^2+covariance3x3(3,3).*use_point(2).^2+a_d.^2.*use_noise(2,2).*use_point(1).^2+a_d.^2.*use_noise(1,1).*use_point(2).^2-a_d.*use_noise(2,2).*use_point(3).*use_point(1).*2.0+a_d.*use_noise(2,1).*use_point(3).*use_point(2).*2.0-a_d.^2.*use_noise(2,1).*use_point(1).*use_point(2).*2.0).*(-1.0./2.0))./(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2))).*1.0./sqrt_cfun(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*1.0./sqrt_cfun(-(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2))./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))).*1.0./(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)).^2.*(a_d.^2.*use_noise(2,1).^4.*covariance3x3(3,3)+use_noise(1,1).*use_noise(2,2).^2.*covariance3x3(3,3).^2-use_noise(2,1).^2.*use_noise(2,2).*covariance3x3(3,3).^2+a_d.^4.*use_noise(2,1).^4.*mean(1).^2+a_d.^2.*use_noise(2,1).^4.*use_point(3).^2+use_noise(2,2).^2.*covariance3x3(3,3).^2.*mean(1).^2+use_noise(2,2).^2.*covariance3x3(3,3).^2.*use_point(1).^2+use_noise(2,1).^2.*covariance3x3(3,3).^2.*use_point(2).^2+a_d.^2.*use_noise(1,1).^2.*use_noise(2,2).^2.*covariance3x3(3,3)+a_d.^4.*use_noise(1,1).^2.*use_noise(2,2).^2.*mean(1).^2+a_d.^2.*use_noise(1,1).^2.*use_noise(2,2).^2.*use_point(3).^2-a_d.^3.*use_noise(2,1).^4.*mean(1).*use_point(3).*2.0-use_noise(2,2).^2.*covariance3x3(3,3).^2.*mean(1).*use_point(1).*2.0+use_noise(2,1).*use_noise(2,2).*covariance3x3(3,3).^2.*mean(1).*use_point(2).*2.0-use_noise(2,1).*use_noise(2,2).*covariance3x3(3,3).^2.*use_point(1).*use_point(2).*2.0-a_d.^2.*use_noise(1,1).*use_noise(2,1).^2.*use_noise(2,2).*covariance3x3(3,3).*2.0-a_d.^2.*use_noise(2,1).^3.*covariance3x3(3,3).*mean(1).*use_point(2).*2.0-a_d.^4.*use_noise(1,1).*use_noise(2,1).^2.*use_noise(2,2).*mean(1).^2.*2.0+a_d.^2.*use_noise(1,1).*use_noise(2,2).^2.*covariance3x3(3,3).*mean(1).^2.*2.0-a_d.^2.*use_noise(2,1).^2.*use_noise(2,2).*covariance3x3(3,3).*mean(1).^2.*2.0-a_d.^2.*use_noise(1,1).*use_noise(2,1).^2.*use_noise(2,2).*use_point(3).^2.*2.0-a_d.^3.*use_noise(1,1).^2.*use_noise(2,2).^2.*mean(1).*use_point(3).*2.0+a_d.*use_noise(2,1).^3.*covariance3x3(3,3).*use_point(3).*use_point(2).*2.0-a_d.*use_noise(1,1).*use_noise(2,2).^2.*covariance3x3(3,3).*mean(1).*use_point(3).*2.0+a_d.*use_noise(2,1).^2.*use_noise(2,2).*covariance3x3(3,3).*mean(1).*use_point(3).*2.0+a_d.*use_noise(1,1).*use_noise(2,2).^2.*covariance3x3(3,3).*use_point(3).*use_point(1).*2.0-a_d.*use_noise(2,1).^2.*use_noise(2,2).*covariance3x3(3,3).*use_point(3).*use_point(1).*2.0+a_d.^3.*use_noise(1,1).*use_noise(2,1).^2.*use_noise(2,2).*mean(1).*use_point(3).*4.0-a_d.^2.*use_noise(1,1).*use_noise(2,2).^2.*covariance3x3(3,3).*mean(1).*use_point(1).*2.0+a_d.^2.*use_noise(2,1).^2.*use_noise(2,2).*covariance3x3(3,3).*mean(1).*use_point(1).*2.0-a_d.*use_noise(1,1).*use_noise(2,1).*use_noise(2,2).*covariance3x3(3,3).*use_point(3).*use_point(2).*2.0+a_d.^2.*use_noise(1,1).*use_noise(2,1).*use_noise(2,2).*covariance3x3(3,3).*mean(1).*use_point(2).*2.0).*2.5e-1i-1.0./pi_.^(3.0./2.0).*1.0./sqrt(covariance3x3(3,3)).*sqrt(pi).*exp(((use_noise(2,2).*use_point(3).^2+covariance3x3(3,3).*use_point(2).^2+a_d.^2.*use_noise(2,2).*use_point(1).^2+a_d.^2.*use_noise(1,1).*use_point(2).^2-a_d.*use_noise(2,2).*use_point(3).*use_point(1).*2.0+a_d.*use_noise(2,1).*use_point(3).*use_point(2).*2.0-a_d.^2.*use_noise(2,1).*use_point(1).*use_point(2).*2.0).*(-1.0./2.0))./(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2))).*erf_rfun((sqrt(2.0).*1.0./sqrt_cfun(-(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2))./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))).*(a_d.*use_noise(2,1).^2.*use_point(3)-use_noise(2,2).*covariance3x3(3,3).*use_point(1)+use_noise(2,1).*covariance3x3(3,3).*use_point(2)-a_d.*use_noise(1,1).*use_noise(2,2).*use_point(3)).*5.0e-1i)./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))).*1.0./sqrt_cfun(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*1.0./sqrt_cfun(-(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2))./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))).*1.0./(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)).^2.*(a_d.^2.*use_noise(2,1).^4.*covariance3x3(3,3)+use_noise(1,1).*use_noise(2,2).^2.*covariance3x3(3,3).^2-use_noise(2,1).^2.*use_noise(2,2).*covariance3x3(3,3).^2+a_d.^4.*use_noise(2,1).^4.*mean(1).^2+a_d.^2.*use_noise(2,1).^4.*use_point(3).^2+use_noise(2,2).^2.*covariance3x3(3,3).^2.*mean(1).^2+use_noise(2,2).^2.*covariance3x3(3,3).^2.*use_point(1).^2+use_noise(2,1).^2.*covariance3x3(3,3).^2.*use_point(2).^2+a_d.^2.*use_noise(1,1).^2.*use_noise(2,2).^2.*covariance3x3(3,3)+a_d.^4.*use_noise(1,1).^2.*use_noise(2,2).^2.*mean(1).^2+a_d.^2.*use_noise(1,1).^2.*use_noise(2,2).^2.*use_point(3).^2-a_d.^3.*use_noise(2,1).^4.*mean(1).*use_point(3).*2.0-use_noise(2,2).^2.*covariance3x3(3,3).^2.*mean(1).*use_point(1).*2.0+use_noise(2,1).*use_noise(2,2).*covariance3x3(3,3).^2.*mean(1).*use_point(2).*2.0-use_noise(2,1).*use_noise(2,2).*covariance3x3(3,3).^2.*use_point(1).*use_point(2).*2.0-a_d.^2.*use_noise(1,1).*use_noise(2,1).^2.*use_noise(2,2).*covariance3x3(3,3).*2.0-a_d.^2.*use_noise(2,1).^3.*covariance3x3(3,3).*mean(1).*use_point(2).*2.0-a_d.^4.*use_noise(1,1).*use_noise(2,1).^2.*use_noise(2,2).*mean(1).^2.*2.0+a_d.^2.*use_noise(1,1).*use_noise(2,2).^2.*covariance3x3(3,3).*mean(1).^2.*2.0-a_d.^2.*use_noise(2,1).^2.*use_noise(2,2).*covariance3x3(3,3).*mean(1).^2.*2.0-a_d.^2.*use_noise(1,1).*use_noise(2,1).^2.*use_noise(2,2).*use_point(3).^2.*2.0-a_d.^3.*use_noise(1,1).^2.*use_noise(2,2).^2.*mean(1).*use_point(3).*2.0+a_d.*use_noise(2,1).^3.*covariance3x3(3,3).*use_point(3).*use_point(2).*2.0-a_d.*use_noise(1,1).*use_noise(2,2).^2.*covariance3x3(3,3).*mean(1).*use_point(3).*2.0+a_d.*use_noise(2,1).^2.*use_noise(2,2).*covariance3x3(3,3).*mean(1).*use_point(3).*2.0+a_d.*use_noise(1,1).*use_noise(2,2).^2.*covariance3x3(3,3).*use_point(3).*use_point(1).*2.0-a_d.*use_noise(2,1).^2.*use_noise(2,2).*covariance3x3(3,3).*use_point(3).*use_point(1).*2.0+a_d.^3.*use_noise(1,1).*use_noise(2,1).^2.*use_noise(2,2).*mean(1).*use_point(3).*4.0-a_d.^2.*use_noise(1,1).*use_noise(2,2).^2.*covariance3x3(3,3).*mean(1).*use_point(1).*2.0+a_d.^2.*use_noise(2,1).^2.*use_noise(2,2).*covariance3x3(3,3).*mean(1).*use_point(1).*2.0-a_d.*use_noise(1,1).*use_noise(2,1).*use_noise(2,2).*covariance3x3(3,3).*use_point(3).*use_point(2).*2.0+a_d.^2.*use_noise(1,1).*use_noise(2,1).*use_noise(2,2).*covariance3x3(3,3).*mean(1).*use_point(2).*2.0).*2.5e-1i);

% Scaling back the integral
value = value / use_length^4;

% Normalization ("total_probability" is already externally available and thus supplied as argument to optimize runtime)
value = value / total_probability;

end