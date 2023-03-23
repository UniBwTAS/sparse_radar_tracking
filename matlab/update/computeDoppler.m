function [ doppler ] = computeDoppler( egoPosition, egoYaw, egoSpeed, egoYawrate, targetPosition, targetYaw, targetSpeed, targetYawrate, detectionPosition, radarPosition)
%computeDoppler Computes the speed between two points on CTRV-parametrized rigid bodies
%   Computes the speed (range derivative) of two points, where each is located on a moving body with CTRV dynamics.
%   This function targets Doppler-radar sensors for kinematic filtering.
%   The two bodies are named "ego" and "target".
%   The CTRV description of both bodies is given by:
%       Position:   the kinematic reference point (pivot point) of the body in a world coordinate system
%       Yaw:        the orientation of the body in the world coordinate system
%       Speed:      the speed of the body along its orientation in the world coordinate system
%       Yawrate:    the derivate of "Yaw"
%   The point "radarPosition" is kinematically linked (attached) to the "ego"-body but also given in the world coordinate system.
%   The point "detectionPosition" is kinematically linked (attached) to the "target"-body but also ginven in the world coordinate system.
%   The return value "doppler" provides the scalar speed (= range derivate) the two points are approaching (negative sign) or moving away (positive sign).

% Orientation of the detection originating from the sensor (Eq. 17)
omega_r = atan2(detectionPosition(2) - radarPosition(2), detectionPosition(1) - radarPosition(1));

% Speed of the requested point z (= "detectionPosition") on the target (Eq. 16)
v_D = [ cos(targetYaw) * targetSpeed - targetYawrate * (detectionPosition(2) - targetPosition(2));
        sin(targetYaw) * targetSpeed + targetYawrate * (detectionPosition(1) - targetPosition(1))];

% Speed of the sensor on the ego (Eq. 15)
v_R = [ cos(egoYaw) * egoSpeed - egoYawrate * (radarPosition(2) - egoPosition(2));
        sin(egoYaw) * egoSpeed + egoYawrate * (radarPosition(1) - egoPosition(1))];

% Projection of the speed difference into the sensor frame (Eq. 18)
doppler = [cos(omega_r) sin(omega_r)] * (v_D - v_R);

end

%% Notes
% Acknowledgements: Dominik Kellner [36]