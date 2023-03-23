%% Script Config
addpath('update');

%% Example Input
% This sample input is exemplary and only mimics the original sensor data.
% The sensor calibration is also exemplary.
% Scenario 1, Measurement Epoch 4091, Track Time 53.051s (Turn Manoeuvre)

% Prior Target State (CTRV + length + width)
x_prior = zeros(7,1);
x_prior(1) =   48.46; % x in m
x_prior(2) = -128.72; % y in m
x_prior(3) =   7.704; % yaw in rad
x_prior(4) =    8.46; % speed in m/s
x_prior(5) =  -0.245; % yaw rate in rad/s
x_prior(6) =    4.89; % length in m
x_prior(7) =    1.83; % width in m

P_prior = zeros(7,7); % uncertainty matrix for x_prior
P_prior(1:7,1:7) = ...
    [ 0.356 0.0408 -0.0752 0.0926 -0.0485 -0.000605 -1.66e-05;          ...
      0.0408 0.0253 -0.0103 0.0408 -0.00922 -7.54e-05 2.45e-06;         ...
     -0.0752 -0.0103 0.0233 -0.0352 0.0247 0.000124 9.55e-07;           ...
      0.0926 0.0408 -0.0352 0.555 -0.0451 -0.000139 1.79e-06;           ...
     -0.0485 -0.00922 0.0247 -0.0451 0.0871 7.94e-05 5.65e-07;          ...
     -0.000605 -7.54e-05 0.000124 -0.000139 7.94e-05 0.00155 -4.35e-06; ...
     -1.66e-05 2.45e-06 9.55e-07 1.79e-06 5.65e-07 -4.35e-06 9.86e-05];

% Ego State (CTRV)
egoState = zeros(5,1);
egoState(1) =   51.19; % x in m
egoState(2) = -149.81; % y in m
egoState(3) = 341.286; % yaw in rad
egoState(4) =   10.05; % speed in m/s
egoState(5) =  -0.263; % yaw rate in m

% Mounting Pose of Current Sensor
% (in vehicle coordinates, middle of rear axis is x=0m, y=0m)
sensorPose = zeros(6,1);
sensorPose(1) = 3.40;           % x in m
sensorPose(2) = -0.85;          % y in m
sensorPose(3) = NaN;            % z in m (unused)
sensorPose(4) = NaN;            % roll in rad (unused)
sensorPose(5) = NaN;            % pitch in rad (unused)
sensorPose(6) = deg2rad(-70);   % yaw in rad

% Detection Measurement
detectionMeasurement = zeros(4,2); % column 1 = measurements, column2 = measurement uncertainty

% yaw angle measurement in rad
detectionMeasurement(1,1) = 0.896;          % exemplary measurement
detectionMeasurement(1,2) = deg2rad(4)^2;   % exemplary uncertainty

% range measurement in m (any bandwidth correction terms have to be applied before)
detectionMeasurement(2,1) = 16.35; % exemplary measurement
detectionMeasurement(2,2) = 0.3^2; % exemplary uncertainty

% amplitude measurement in dB (unused)
detectionMeasurement(3,1) = 5; % exemplary measurement
detectionMeasurement(3,2) = 2; % exemplary uncertainty

% radial speed measurement in m/s
detectionMeasurement(4,1) = -1.58;  % exemplary measurement
detectionMeasurement(4,2) = 0.05^2; % exemplary uncertainty

%% State Update
[x_out, P_out] = update(x_prior, P_prior, egoState, sensorPose, detectionMeasurement);
