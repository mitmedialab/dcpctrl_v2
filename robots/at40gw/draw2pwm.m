function [ pwm, err ] = draw2pwm( robot, d_raw  )
%DRAW2PWM Generic function to convert raw joint velocities to PWM signals
%for the AT40GW.

%{
[ pwm ] = draw2pwm( robot, d_raw, n )
Julian Leland, MIT Media Lab, 2017-02-08


%}

% Get PWM max and min limits
pwm_max = robot.Joint(n).PWMLim.Max;
pwm_min = robot.Joint(n).PWMLim.Min;
pwm_dbup = robot.Joint(n).PWMLim.DBUpper;
pwm_dblo = robot.Joint(n).PWMLim.DBLower;
pwm_zero = robot.PWMZero;
movethresh = robot.Joint(n).moveThresh;

% Get upper and lower knee limits
knee_upper = robot.Joint(n).VelMap.PosKnee; % Highest possible sensor velocity value
knee_lower = robot.Joint(n).VelMap.NegKnee; % Lowest possible sensor velocity value

pwm = zeros(size(d_raw));

posi = d_raw > ;
nega = d_raw < -10;
zer = d_raw >= -10 & d_raw <= 10;

pwm(posi) = mapRange(d_raw(posi),10,knee_upper,pwm_dbup,pwm_max);
pwm(nega) = mapRange(d_raw(nega),-10,knee_lower,pwm_dblo,pwm_min);
pwm(zer) = robot.PWMZero;
pwm = min(max(pwm, pwm_min), pwm_max);

end