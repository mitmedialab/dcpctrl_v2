function Q_jointJ3 = raw2jointJ4Abs_at40gw(robot, Q_rawJ3)
% RAW2JOINTJ3ABS_AT40GW Converts raw absolute sensor values to joint
% positions for J3 on AT40GW

%{
raw2jointJ3Abs_at40gw.m
Julian Leland, MIT Media Lab, 2017-05-23

This function implements raw2joint for *just* the J3 joint of the AT40GW
%}
Q_jointJ3 = zeros(size(Q_rawJ3));

% Joint 3: Voltages to degrees
a = robot.Joint(3).PosSensParams.a_len; % Distance between joint axis and cylinder base axis
b = robot.Joint(3).PosSensParams.b_len; % Distance between joint axis and cylinder attachment point axis
c = robot.Joint(3).PosSensParams.c_len; % Length of cylinder at J_Angle = 0
jointV_Zero = robot.Joint(3).PosSensParams.ZeroVoltage; % Sensor voltage at zero position
jointV_Max = robot.Joint(3).PosLim.Max; % Maximum sensor voltage
jointV_Min = robot.Joint(3).PosLim.Min; % Minimum sensor voltage
sensLen = robot.Joint(3).PosSensParams.SensLen; % Length of joint sensor (allows us to figure out where we are along joint)

% We need to make the orientation of our max/min sensor values match the
% orientation of the joint we're working on.
sensVMax = 10*sign(jointV_Max); 
sensVMin = 10*sign(jointV_Min);

% Find position along sensor of joint zero, relative to center
sensP_Zero = ((jointV_Zero-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);

% At current voltage, how far away from jointP_Zero are we?
sensP_Cur = ((Q_rawJ3(:)-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);
dSensP = sensP_Zero - sensP_Cur;
J_Len = c - dSensP;
Q_jointJ3(:) = (acosd(((c^2)-(a^2)-(b^2))/(-2*a*b))-acosd(((J_Len.^2)-(a^2)-(b^2))/(-2*a*b)));