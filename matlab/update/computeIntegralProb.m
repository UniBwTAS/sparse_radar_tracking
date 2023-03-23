function [value] = computeIntegralProb(covariance3x3, pointStart, pointEnd, point)
%COMPUTEINTEGRALPROB Integrated normal distribution along a 3D line
%   covariance3x3:  3D covariance matrix (2D positional, 1D kinematical)
%   pointStart:     3D point of on end point of the line
%   pointEnd:       3D point of the other end point of the line
%   point:          requested 3D point, for which the integrated likelihood should be computed
%   Returns the integrated likelihood for "point".

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
value = real((1.0./pi_.^(3.0./2.0).*1.0./sqrt(covariance3x3(3,3)).*sqrt(pi).*exp((a_d.^2.*use_noise(1,1).^2.*use_noise(2,2).^2.*use_point(3).^2)./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)).*2.0)).*exp((use_noise(2,2).^2.*covariance3x3(3,3).*use_point(1).^2)./((use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)).*2.0)).*exp((use_noise(2,1).^2.*covariance3x3(3,3).*use_point(2).^2)./((use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)).*2.0)).*exp(-(a_d.^2.*use_noise(1,1).*use_noise(2,1).^2.*use_noise(2,2).*use_point(3).^2)./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)))).*exp((a_d.*use_noise(2,1).^3.*use_point(3).*use_point(2))./((use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)))).*exp((a_d.^2.*use_noise(2,1).^4.*use_point(3).^2)./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)).*2.0)).*exp(-(use_noise(2,2).*use_point(1).^2)./(use_noise(1,1).*use_noise(2,2).*2.0-use_noise(2,1).^2.*2.0)).*exp(-(use_noise(1,1).*use_point(2).^2)./(use_noise(1,1).*use_noise(2,2).*2.0-use_noise(2,1).^2.*2.0)).*exp(-(use_noise(2,1).*use_noise(2,2).*covariance3x3(3,3).*use_point(1).*use_point(2))./((use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)))).*exp((use_noise(2,1).*use_point(1).*use_point(2))./(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2)).*exp((use_noise(2,1).^2.*use_point(3).^2)./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*2.0)).*exp((use_noise(1,1).*use_noise(2,2).*use_point(3).^2.*(-1.0./2.0))./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))).*exp((a_d.*use_noise(1,1).*use_noise(2,2).^2.*use_point(3).*use_point(1))./((use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)))).*exp(-(a_d.*use_noise(2,1).^2.*use_noise(2,2).*use_point(3).*use_point(1))./((use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)))).*exp(-(a_d.*use_noise(1,1).*use_noise(2,1).*use_noise(2,2).*use_point(3).*use_point(2))./((use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2)))).*1.0./sqrt_cfun(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*(erf_rfun((sqrt(2.0).*1.0./sqrt_cfun(-(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2))./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))).*(a_d.*use_noise(2,1).^2.*use_point(3).*1i-a_d.^2.*use_length.*use_noise(2,1).^2.*1i+use_length.*use_noise(2,2).*covariance3x3(3,3).*1i-use_noise(2,2).*covariance3x3(3,3).*use_point(1).*1i+use_noise(2,1).*covariance3x3(3,3).*use_point(2).*1i-a_d.*use_noise(1,1).*use_noise(2,2).*use_point(3).*1i+a_d.^2.*use_length.*use_noise(1,1).*use_noise(2,2).*1i))./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*2.0)).*1i-erf_rfun((sqrt(2.0).*1.0./sqrt_cfun(-(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2))./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))).*(a_d.*use_noise(2,1).^2.*use_point(3).*1i-use_noise(2,2).*covariance3x3(3,3).*use_point(1).*1i+use_noise(2,1).*covariance3x3(3,3).*use_point(2).*1i-a_d.*use_noise(1,1).*use_noise(2,2).*use_point(3).*1i))./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2).*2.0)).*1i).*1.0./sqrt_cfun(-(use_noise(2,2).*covariance3x3(3,3)-a_d.^2.*use_noise(2,1).^2+a_d.^2.*use_noise(1,1).*use_noise(2,2))./(covariance3x3(3,3).*(use_noise(1,1).*use_noise(2,2)-use_noise(2,1).^2))))./4.0 / use_length);

end

