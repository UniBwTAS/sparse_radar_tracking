function [ x_posterior, P_posterior ] = update( x_prior, P_prior, egoState, sensorLocationInEgo, detectionMeasurement)
%update State update of "Probabilistic Vehicle Tracking with Sparse Radar Detection Measurements" (JAIF Special Issue on Automotive Radar Perception Systems)
%   Performs a state update with a single radar detection measurement.
%   More details can be found in the paper. Example arguments are given in "run_example.m".
%   x_prior:                Prior state estimate mean (7x1: 5x1 CTRV (x, y, yaw, speed, yawrate) given in world coordinates, length, width)  in SI-units (using radiant)
%   P_prior:                Prior state estimate uncertainty (7x7)
%   egoState:               CTRV-state of the ego vehicle
%   sensorLocationInEgo:    the mounting location of the active sensor in ego coordinates (x,y,z,roll,pitch,yaw but only x,y,yaw are supported)
%   detectionMeasurement:   the detection measurement (yaw angle, range, amplitude, radial speed) x (measurement, uncertainty) in SI-units (using radiant)
%   configuration:          the configuration of the sensor and object model (see "run_example.m")
%   x_posterior:            Posterior state estimate mean (structure equals "x_prior")
%   P_posterior:            Posterior state estimate uncertainty (structure equals "P_prior")

%% Notes
% - This code is a reproduction of the algorithm presented in "Probabilistic Vehicle Tracking with Sparse Radar Detection Measurements".
% - The function arguments are also outlined in "run_example.m".
% - The sample input is exemplary and only mimics the original sensor data.
% - The sensor calibration is also exemplary. Any sensor-affected correction terms have to be applied to the input measurement data.
% - This code targets comprehension and uses structures and dynamic arrays for better readability.
%   Unrolling the for-loops and using inline variables instead HEAVILY boosts the MATLAB runtime performance and will provide the claimed performance.

%% Constants and Indices Definitions
FL = 1; % front left
FR = 2; % front right
BL = 3; % rear  left
BR = 4; % rear  right

sF = 1; % front side
sB = 2; % rear  side
sL = 3; % left  side
sR = 4; % right side

%% Helper Functions
% Transform a point "position" given in target coordinates to world coordinates
% using the target state "state_x".
transformFromTargetToWorldCoordinates = @(state_x,position) state_x(1:2) + R(state_x(3)) * position;

% Computes the distance from the line "[AB]" to the point "P" (not normalized, as only the sign (left/right of line) matters here) (Eq. 22)
getDistance = @(A,B,P) (P(1) - A(1)) * (B(2) - A(2)) - (P(2) - A(2)) * (B(1) - A(1));

% Returns the n-th element of the input (workaround for "fun(x)(n)")
getNthElement = @(in,n) in(n);

%% Definition of the Object Model (Table II)
% The following object definitions are given as lambda functions and take
% any desired state hypothesis as argument. The measurement matrix is later
% computed using a complex jet differentiation.

% Corners
om.corners.detectionRate.base = 1;
om.corners.detectionRate.visibilityConstraint = @(isVisible) real(logical(isVisible));

om.corner{FL}.positionMean = @(state_x) [+0.65 * state_x(6); +0.25 * state_x(7)];
om.corner{FR}.positionMean = @(state_x) [+0.65 * state_x(6); -0.25 * state_x(7)];
om.corner{BL}.positionMean = @(state_x) [-0.2  * state_x(6); +0.35 * state_x(7)];
om.corner{BR}.positionMean = @(state_x) [-0.2  * state_x(6); -0.35 * state_x(7)];

om.corner{FL}.positionUncertainty = R(-1*pi/4) * diag([0.15;0.05]).^2 * R(-1*pi/4)';
om.corner{FR}.positionUncertainty = R(-3*pi/4) * diag([0.15;0.05]).^2 * R(-3*pi/4)';
om.corner{BL}.positionUncertainty = R(+1*pi/4) * diag([0.15;0.05]).^2 * R(+1*pi/4)';
om.corner{BR}.positionUncertainty = R(+3*pi/4) * diag([0.15;0.05]).^2 * R(+3*pi/4)';

% Wheels
om.wheels.detectionRate.base = 0.66;
om.wheels.detectionRate.visibilityConstraint = @(isVisible) 1*real(logical(isVisible)) + 0.3*real(~logical(isVisible));

om.wheel{FL}.positionMean = @(state_x) [0.5 * state_x(6); +0.5 * state_x(7) - 0.15];
om.wheel{FR}.positionMean = @(state_x) [0.5 * state_x(6); -0.5 * state_x(7) + 0.15];
om.wheel{BL}.positionMean = @(state_x) [0;                +0.5 * state_x(7) - 0.15];
om.wheel{BR}.positionMean = @(state_x) [0;                -0.5 * state_x(7) + 0.15];

om.wheel{FL}.positionUncertainty = diag([0.2;0.1]).^2;
om.wheel{FR}.positionUncertainty = diag([0.2;0.1]).^2;
om.wheel{BL}.positionUncertainty = diag([0.2;0.1]).^2;
om.wheel{BR}.positionUncertainty = diag([0.2;0.1]).^2;

% Sides
om.sides.detectionRate.basePerRad = rad2deg(0.29);
om.sides.detectionRate.visibilityConstraint = @(isVisible) real(logical(isVisible));

om.side{sF}.positionA = @(state_x) [+0.67 * state_x(6); -0.125 * state_x(7)       ];
om.side{sF}.positionB = @(state_x) [+0.67 * state_x(6); +0.125 * state_x(7)       ];
om.side{sB}.positionA = @(state_x) [-0.2  * state_x(6); +0.15  * state_x(7)       ];
om.side{sB}.positionB = @(state_x) [-0.2  * state_x(6); -0.15  * state_x(7)       ];
om.side{sL}.positionA = @(state_x) [+0.6  * state_x(6); +0.5   * state_x(7) - 0.15];
om.side{sL}.positionB = @(state_x) [-0.15 * state_x(6); +0.5   * state_x(7) - 0.15];
om.side{sR}.positionA = @(state_x) [-0.15 * state_x(6); -0.5   * state_x(7) + 0.15];
om.side{sR}.positionB = @(state_x) [+0.6 *  state_x(6); -0.5   * state_x(7) + 0.15];

om.side{sF}.positionUncertainty = R(-pi/2) * diag([0.0;0.05]).^2 * R(-pi/2)';
om.side{sB}.positionUncertainty = R(+pi/2) * diag([0.0;0.05]).^2 * R(+pi/2)';
om.side{sL}.positionUncertainty =            diag([0.0;0.05]).^2            ;
om.side{sR}.positionUncertainty =            diag([0.0;0.05]).^2            ;

% Body
om.body.detectionRate.base = 0.1;

% Clutter
om.clutter.detectionRate.base = 0.01;

%% Sensor Model
sm.detectionRate.reference = 1;
% This calibration value is exemplary. As the component association is performed relatively,
% all components are at an equal distance and the detection rate of the sensor mainly depends
% on the distance, this value does not show a significant effect on the filtering.
% However, it has to be given correctly if the expected number of detections needs to be computed.
% (This example roughly considers the relation to the clutter likelihood.)

%% Unwrap Arguments

% Ego Vehicle State (in World Coordinates)
egoPositionInWorld = egoState(1:2);
egoYawInWorld      = egoState(3);
egoSpeedInWorld    = egoState(4);
egoYawrateInWorld  = egoState(5);

% Sensor Mounting Location (in Ego Coordinates)
sensorPositionInEgo = sensorLocationInEgo(1:2);
sensorYawInEgo      = sensorLocationInEgo(6);

% Detection Measurement (in Sensor Coordinates)
angleMeasurementInSensor       = detectionMeasurement(1,1);
angleMeasurementInSensor_noise = detectionMeasurement(1,2);

rangeMeasurementInSensor       = detectionMeasurement(2,1);
rangeMeasurementInSensor_noise = detectionMeasurement(2,2);

radialSpeedMeasurementInSensor       = detectionMeasurement(4,1);
radialSpeedMeasurementInSensor_noise = detectionMeasurement(4,2);

%% Coordinate Transformations (Section II)

detectionAngleInEgo = angleMeasurementInSensor + sensorYawInEgo;

detectionPositionInEgo = sensorPositionInEgo + [ ...
    cos(detectionAngleInEgo) * rangeMeasurementInSensor; ...
    sin(detectionAngleInEgo) * rangeMeasurementInSensor ...
    ];

detectionPositionInWorld        = egoPositionInWorld + R(egoYawInWorld) * detectionPositionInEgo;
detectionMeasurement            = [detectionPositionInWorld; radialSpeedMeasurementInSensor];

detectionPositionInSensor_noise = diag([rangeMeasurementInSensor_noise, (2 * rangeMeasurementInSensor * tan(sqrt(angleMeasurementInSensor_noise)/2))^2]);
detectionPositionInWorld_noise  = R(angleMeasurementInSensor + sensorYawInEgo + egoYawInWorld) * detectionPositionInSensor_noise * R(angleMeasurementInSensor + sensorYawInEgo + egoYawInWorld)';
detectionMeasurement_noise      = dUp(detectionPositionInWorld_noise,3) + diag([0,0,radialSpeedMeasurementInSensor_noise]);
     
sensorPositionInWorld = egoPositionInWorld + R(egoYawInWorld) * sensorPositionInEgo;

%% Input State Estimate
% load variables
x = x_prior;
P = P_prior;

%% State-dependent Functions

% Returns the radial speed of a point of the target
getDoppler = @(state_x, source) computeDoppler(egoPositionInWorld, egoYawInWorld, egoSpeedInWorld, egoYawrateInWorld, state_x(1:2), state_x(3), state_x(4), state_x(5), source, egoPositionInWorld + R(egoYawInWorld) * sensorPositionInEgo);

% Lambda: returns if side "indice" is facing the sensor (Eq. 23)
isSideVisible = @(iC) getDistance(transformFromTargetToWorldCoordinates(x, om.side{iC}.positionA(x)), transformFromTargetToWorldCoordinates(x, om.side{iC}.positionB(x)), sensorPositionInWorld) > 0;

%% Process Vehicle Corners

% Lambda: returns if corner "indice" is facing the sensor
isCornerFacing = @(indice) ((indice == FL) && isSideVisible(sF) && isSideVisible(sL)) || ...
                           ((indice == FR) && isSideVisible(sF) && isSideVisible(sR)) || ... 
                           ((indice == BL) && isSideVisible(sB) && isSideVisible(sL)) || ...
                           ((indice == BR) && isSideVisible(sB) && isSideVisible(sR));

% For all four corners do...
for iC = [FL,FR,BL,BR]

    % Compute detection rate (Eq. 14)
    detectionRate_corner{iC} = ...
        sm.detectionRate.reference      * ...                               % Reference Rate (Eq. 7)
        om.corners.detectionRate.base   * ...                               % Base Rate (Table II)
        om.corners.detectionRate.visibilityConstraint(isCornerFacing(iC));  % Visibility Constraint (Eq. 13)

    % Lambda function returning the expected measurement
    getExpectedMeasurementInWorld_corner{iC} = @(state_x) transformFromTargetToWorldCoordinates(state_x, om.corner{iC}.positionMean(state_x));
    
    % Compute the expected measurement and the linearized measurement matrix (Eq. 31, Eq. 32)
    [y_corner{iC}, C_corner{iC}] = jaccsdg(@(state_x, par) [getExpectedMeasurementInWorld_corner{iC}(state_x); getDoppler(state_x, par)], x, detectionPositionInWorld);

    % Compute the innovation covariance matrix (Eq. 33)
    S_corner{iC} = C_corner{iC} * P * C_corner{iC}' + dUp(R(x(3)) * om.corner{iC}.positionUncertainty * R(x(3))',3) + detectionMeasurement_noise;

    % Compute the detection likelihood (Eq. 34)
    gamma_corner{iC} = detectionRate_corner{iC} * ((2*pi)^3 *det(S_corner{iC}))^(-0.5) * exp(-0.5*(y_corner{iC}-detectionMeasurement)'*inv(S_corner{iC})*(y_corner{iC}-detectionMeasurement));
    
    % Compute the Kalman gain (Eq. 57)
    K_corner{iC} = P * C_corner{iC}' / S_corner{iC};

    % Compute the component-wise state update (Eq. 54)
    x_corner{iC} = x + K_corner{iC} * (detectionMeasurement - y_corner{iC});
end

%% Process Vehicle Wheels

% Lambda: returns if wheel "indice" is facing the sensor
isWheelFacing = @(indice) (mod(indice,2) && isSideVisible(sL)) || ((~mod(indice,2)) && isSideVisible(sR));

% For all four wheels do...
for iC = [FL,FR,BL,BR]

    % Compute detection rate (Eq. 20)
    detectionRate_wheel{iC} = ...
        sm.detectionRate.reference      * ...                               % Reference Rate (Eq. 7)
        om.wheels.detectionRate.base    * ...                               % Base Rate (Table II)
        om.wheels.detectionRate.visibilityConstraint(isWheelFacing(iC));    % Visibility Constraint (Eq. 19)

    % Lambda function returning the expected measurement
    getExpectedMeasurementInWorld_wheel{iC} = @(state_x) transformFromTargetToWorldCoordinates(state_x, om.wheel{iC}.positionMean(state_x));

    % Compute the expected measurement and the linearized measurement matrix (Eq. 35, Eq. 36)
    [y_wheel{iC}, C_wheel{iC}] = jaccsd(getExpectedMeasurementInWorld_wheel{iC},x);

    % Compute the innovation covariance matrix (Eq. 37)
    S_wheel{iC} = C_wheel{iC} * P * C_wheel{iC}' + R(x(3)) * om.wheel{iC}.positionUncertainty * R(x(3))' + detectionPositionInWorld_noise;

    % Compute the detection likelihood (Eq. 38)
    gamma_wheel{iC} = detectionRate_wheel{iC} * ((2*pi)^2 *det(S_wheel{iC}))^(-0.5) * exp(-0.5*(y_wheel{iC}-detectionPositionInWorld)'*inv(S_wheel{iC})*(y_wheel{iC}-detectionPositionInWorld));
    
    % Compute the Kalman gain (Eq. 57)
    K_wheel{iC} = P * C_wheel{iC}' / S_wheel{iC};

    % Compute the component-wise state update (Eq. 54)
    x_wheel{iC} = x + K_wheel{iC} * (detectionPositionInWorld - y_wheel{iC});

end

%% Process Vehicle Sides

% For all four sides do...
for iC = [sF,sB,sL,sR]

    % Get expected positional and kinematic measurements at both end points of the side
    expectedPositionInWorld_side{iC}.A = transformFromTargetToWorldCoordinates(x, om.side{iC}.positionA(x));
    expectedPositionInWorld_side{iC}.B = transformFromTargetToWorldCoordinates(x, om.side{iC}.positionB(x));
    expectedDoppler_side{iC}.A  = getDoppler(x, expectedPositionInWorld_side{iC}.A);
    expectedDoppler_side{iC}.B  = getDoppler(x, expectedPositionInWorld_side{iC}.B);

    % Lambda function returning the expected positional measurement at the midpoint of the side (Eq. 25)
    getExpectedCenterInWorld_side{iC} = @(state_x) transformFromTargetToWorldCoordinates(state_x, (om.side{iC}.positionA(state_x) + om.side{iC}.positionB(state_x)) / 2);
    
    % Compute the expected measurement and the linearized measurement matrix for the midpoint
    [y_side_center{iC}, C_side_center{iC}] = jaccsdg(@(state_x, par) [getExpectedCenterInWorld_side{iC}(state_x); getDoppler(state_x, par)], x, getExpectedCenterInWorld_side{iC}(x));

    % Compute angular (observation) width (Eq. 21)
    angularWidth_side{iC} = abs(myWrapToPi(atan2i(expectedPositionInWorld_side{iC}.B - sensorPositionInWorld) - atan2i(expectedPositionInWorld_side{iC}.A - sensorPositionInWorld)));

    % Compute scattering damping (Eq. 24, Eq. 26)
    scatteringDamping_side{iC} = sin(atan2i(expectedPositionInWorld_side{iC}.B - expectedPositionInWorld_side{iC}.A) - atan2i(y_side_center{iC}(1:2) - sensorPositionInWorld))^2;

    % Compute detection rate (Eq. 27)
    detectionRate_side{iC} = ...
        sm.detectionRate.reference          * ...   % Reference Rate (Eq. 7)
        om.sides.detectionRate.basePerRad   * ...   % Base Rate per radiant (Table II)
        angularWidth_side{iC}               * ...   % Angular observation width
        isSideVisible(iC)                   * ...   % Visibility Constraint (Eq. 23)
        scatteringDamping_side{iC};                 % Scattering Damping

    % Compute the innovation covariance matrix for the midpoint
    S_side_center{iC} = C_side_center{iC} * P * C_side_center{iC}' + dUp(R(x(3)) * om.side{iC}.positionUncertainty * R(x(3))',3)  + detectionMeasurement_noise;

    % Compute the total "stick" detection likelihood (integral of Eq.44 but only over the right factor of Eq. 43)
    gamma_side_stick{iC} = ...
        real(computeIntegralProb( ...
                S_side_center{iC}, ...
                [expectedPositionInWorld_side{iC}.A; expectedDoppler_side{iC}.A], ...
                [expectedPositionInWorld_side{iC}.B; expectedDoppler_side{iC}.B], ...
                detectionMeasurement ...
            ));

    % Compute the detection likelihood (Eq. 44)
    gamma_side{iC} = detectionRate_side{iC} * gamma_side_stick{iC};

    % Numeric checks (necessary for negligible likelihoods close to zero)
    if ~(isnan(gamma_side_stick{iC}) || isinf(gamma_side_stick{iC}) || gamma_side_stick{iC} < 1e-14)

        % Compute the probabilistic "stick" mean "u" (Eq. 51)
        u_side_stick{iC} = ...
            computeIntegralMean( ...
                S_side_center{iC}, ...
                [expectedPositionInWorld_side{iC}.A; expectedDoppler_side{iC}.A], ...
                [expectedPositionInWorld_side{iC}.B; expectedDoppler_side{iC}.B], ...
                detectionMeasurement, ...
                gamma_side_stick{iC} ...    % supplied for runtime optimization (value already available here)
            );
    
        % Compute the probabilistic "stick" mean position, but first in target coordinates (similar to Eq. 52)
        y_side_inTarget{iC} =  u_side_stick{iC} * ...
            (om.side{iC}.positionB(x) - om.side{iC}.positionA(x)) + om.side{iC}.positionA(x);    
    
        % Compute the "spread of means" (right term in integral of Eq. 61)
        P_side_stick_inU{iC} = computeIntegralSpread( ...
            S_side_center{iC}, ...
            [expectedPositionInWorld_side{iC}.A; expectedDoppler_side{iC}.A], ...
            [expectedPositionInWorld_side{iC}.B; expectedDoppler_side{iC}.B], ...
            detectionMeasurement, ...
            u_side_stick{iC}, ...
            gamma_side_stick{iC});
    
        P_side{iC} = P_side_stick_inU{iC} * ...
            ([expectedPositionInWorld_side{iC}.B; expectedDoppler_side{iC}.B] - [expectedPositionInWorld_side{iC}.A; expectedDoppler_side{iC}.A]) * ...
            ([expectedPositionInWorld_side{iC}.B; expectedDoppler_side{iC}.B] - [expectedPositionInWorld_side{iC}.A; expectedDoppler_side{iC}.A])';
    else

        % using dummy values in case likelihood is negligible
        gamma_side_stick{iC} = 0;
        y_side_inTarget{iC} = (om.side{iC}.positionA(x) + om.side{iC}.positionB(x)) / 2;
        P_side{iC} = zeros(3,3);
    end
    
    % Compute the expected measurement and the linearized measurement matrix for the determined side (orthogonal constrain)
    if (iC == sF) || (iC == sB) % TODO transponierung entfernen
        [y_side{iC}, C_side{iC}] = jaccsdg(@(state_x, par) [transformFromTargetToWorldCoordinates(state_x, [getNthElement(om.side{iC}.positionA(state_x),1), par(2)]'); getDoppler(state_x, detectionPositionInWorld)], x, y_side_inTarget{iC});
    else
        [y_side{iC}, C_side{iC}] = jaccsdg(@(state_x, par) [transformFromTargetToWorldCoordinates(state_x, [par(1), getNthElement(om.side{iC}.positionA(state_x),2)]'); getDoppler(state_x, detectionPositionInWorld)], x, y_side_inTarget{iC});
    end

    % Compute the innovation covariance matrix (Eq. 61 using splitting approach of Eq. 59)
    S_side{iC} = C_side{iC} * P * C_side{iC}' + dUp(R(x(3)) * om.side{iC}.positionUncertainty * R(x(3))',3) + P_side{iC} + detectionMeasurement_noise;
    
    % Compute the Kalman gain (Eq. 57)
    K_side{iC} = P * C_side{iC}' / S_side{iC};

    % Compute the component-wise state update (Eq. 54)
    x_side{iC} = x + K_side{iC} * (detectionMeasurement - y_side{iC});
end

%% Process Vehicle Body

% Compute detection rate (Eq. 28)
detectionRate_body = ...
    sm.detectionRate.reference      * ...   % Reference Rate (Eq. 7)
    om.body.detectionRate.base;             % Base Rate (Table II)

% Compute the expected measurement and the linearized measurement matrix (Eq. 46, Eq. 47)
[y_body, C_body] = jaccsdg(@(state_x, par) [getDoppler(state_x, par)], x, detectionPositionInWorld);

% Compute the innovation covariance matrix (Eq. 48)
S_body = C_body * P * C_body' + radialSpeedMeasurementInSensor_noise;

% Compute the detection likelihood (Eq. 49)
gamma_body = detectionRate_body * ((2*pi)^2 *det(S_body))^(-0.5) * exp(-0.5*(y_body-radialSpeedMeasurementInSensor)'*inv(S_body)*(y_body-radialSpeedMeasurementInSensor));

% Compute the Kalman gain (Eq. 57)
K_body   = P * C_body' / S_body;

% Compute the component-wise state update (Eq. 54)
x_body   = x + K_body * (radialSpeedMeasurementInSensor - y_body);

%% Process Clutter

% Detection rate
detectionRate_clutter = ...
    om.clutter.detectionRate.base;          % Base Rate (Section VIII)

% Detection likelihood
gamma_clutter = detectionRate_clutter;

%% Fusion

% Compute the sum of the detection likelihoods of all components (Eq. 50 denominator)
gamma_sum = ...
    gamma_wheel{FL}     + gamma_wheel{FR}   + gamma_wheel{BL}   + gamma_wheel{BR}   + ...
    gamma_corner{FL}    + gamma_corner{FR}  + gamma_corner{BL}  + gamma_corner{BR}  + ...
    gamma_side{sF}      + gamma_side{sR}    + gamma_side{sL}    + gamma_side{sB}    + ...
    gamma_body + ...
    gamma_clutter;

% Compute the association probabilities (Eq. 50)
beta_wheel{FL}  = gamma_wheel{FL}   / gamma_sum;
beta_wheel{FR}  = gamma_wheel{FR}   / gamma_sum;
beta_wheel{BL}  = gamma_wheel{BL}   / gamma_sum;
beta_wheel{BR}  = gamma_wheel{BR}   / gamma_sum;
beta_corner{FL} = gamma_corner{FL}  / gamma_sum;
beta_corner{FR} = gamma_corner{FR}  / gamma_sum;
beta_corner{BL} = gamma_corner{BL}  / gamma_sum;
beta_corner{BR} = gamma_corner{BR}  / gamma_sum;
beta_side{sF}   = gamma_side{sF}    / gamma_sum;
beta_side{sR}   = gamma_side{sR}    / gamma_sum;
beta_side{sL}   = gamma_side{sL}    / gamma_sum;
beta_side{sB}   = gamma_side{sB}    / gamma_sum;
beta_body       = gamma_body        / gamma_sum;
beta_clutter    = gamma_clutter     / gamma_sum;

% Compute fused state estimate mean (Eq. 55)
x_fused = ...
    beta_wheel{FL}  * x_wheel{FL}    + ...
    beta_wheel{FR}  * x_wheel{FR}    + ...
    beta_wheel{BL}  * x_wheel{BL}    + ...
    beta_wheel{BR}  * x_wheel{BR}    + ...
    beta_corner{FL} * x_corner{FL}   + ...
    beta_corner{FR} * x_corner{FR}   + ...
    beta_corner{BL} * x_corner{BL}   + ...
    beta_corner{BR} * x_corner{BR}   + ...
    beta_side{sF}   * x_side{sF}     + ...
    beta_side{sR}   * x_side{sR}     + ...
    beta_side{sL}   * x_side{sL}     + ...
    beta_side{sB}   * x_side{sB}     + ...
    beta_body       * x_body         + ...
    beta_clutter    * x;

% Compute fused state estimate uncertainty (Eq. 56)
P_fused = ...
    beta_wheel{FL}  * (P - K_wheel{FL}   * S_wheel{FL}   * K_wheel{FL}'  + (x_wheel{FL}  - x_fused)*(x_wheel{FL}     - x_fused)') + ...
    beta_wheel{FR}  * (P - K_wheel{FR}   * S_wheel{FR}   * K_wheel{FR}'  + (x_wheel{FR}  - x_fused)*(x_wheel{FR}     - x_fused)') + ...
    beta_wheel{BL}  * (P - K_wheel{BL}   * S_wheel{BL}   * K_wheel{BL}'  + (x_wheel{BL}  - x_fused)*(x_wheel{BL}     - x_fused)') + ...
    beta_wheel{BR}  * (P - K_wheel{BR}   * S_wheel{BR}   * K_wheel{BR}'  + (x_wheel{BR}  - x_fused)*(x_wheel{BR}     - x_fused)') + ...
    beta_corner{FL} * (P - K_corner{FL}  * S_corner{FL}  * K_corner{FL}' + (x_corner{FL} - x_fused)*(x_corner{FL}    - x_fused)') + ...
    beta_corner{FR} * (P - K_corner{FR}  * S_corner{FR}  * K_corner{FR}' + (x_corner{FR} - x_fused)*(x_corner{FR}    - x_fused)') + ...
    beta_corner{BL} * (P - K_corner{BL}  * S_corner{BL}  * K_corner{BL}' + (x_corner{BL} - x_fused)*(x_corner{BL}    - x_fused)') + ...
    beta_corner{BR} * (P - K_corner{BR}  * S_corner{BR}  * K_corner{BR}' + (x_corner{BR} - x_fused)*(x_corner{BR}    - x_fused)') + ...
    beta_side{sF}   * (P - K_side{sF}    * S_side{sF}    * K_side{sF}'   + (x_side{sF}   - x_fused)*(x_side{sF}      - x_fused)') + ...
    beta_side{sR}   * (P - K_side{sR}    * S_side{sR}    * K_side{sR}'   + (x_side{sR}   - x_fused)*(x_side{sR}      - x_fused)') + ...
    beta_side{sL}   * (P - K_side{sL}    * S_side{sL}    * K_side{sL}'   + (x_side{sL}   - x_fused)*(x_side{sL}      - x_fused)') + ...
    beta_side{sB}   * (P - K_side{sB}    * S_side{sB}    * K_side{sB}'   + (x_side{sB}   - x_fused)*(x_side{sB}      - x_fused)') + ...
    beta_body       * (P - K_body        * S_body        * K_body'       + (x_body       - x_fused)*(x_body          - x_fused)') + ...
    beta_clutter    * (P);

%% Output State Estimate

% load variables
x = 1.0*x_fused;
P = 1.0*P_fused;

% enforce symmetry (to avoid numeric divergence)
P = (P+P')/2;   

% return state estimate
x_posterior = x;
P_posterior = P;

end

