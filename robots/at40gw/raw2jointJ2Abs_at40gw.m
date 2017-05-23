function Q_jointJ2 = raw2jointJ2Abs_at40gw(robot, Q_rawJ2)
% RAW2JOINTJ2ABS_AT40GW Converts raw absolute sensor values to joint
% positions for J2 on AT40GW

%{
raw2jointJ2Abs_at40gw.m
Julian Leland, MIT Media Lab, 2017-05-23

This function implements raw2joint for *just* the J2 joint of the AT40GW
%}

Q_jointJ2 = zeros(size(Q_rawJ2));

% Joint 2: Voltages to degrees
a = robot.Joint(2).PosSensParams.a_len; % Distance between joint axis and cylinder base axis
b = robot.Joint(2).PosSensParams.b_len; % Distance between joint axis and cylinder attachment point axis
c = robot.Joint(2).PosSensParams.c_len; % Length of cylinder at J_Angle = 0
jointV_Zero = robot.Joint(2).PosSensParams.ZeroVoltage; % Sensor voltage at zero position
jointV_Max = robot.Joint(2).PosLim.Max; % Maximum sensor voltage
jointV_Min = robot.Joint(2).PosLim.Min; % Minimum sensor voltage
sensLen = robot.Joint(2).PosSensParams.SensLen; % Length of joint sensor (allows us to figure out where we are along joint)

% We need to make the orientation of our max/min sensor values match the
% orientation of the joint we're working on.
sensVMax = 10*sign(jointV_Max); 
sensVMin = 10*sign(jointV_Min);
% Find position along sensor of joint zero, relative to center
sensP_Zero = ((jointV_Zero-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);

% At current voltage, how far away from jointP_Zero are we?
sensP_Cur = ((Q_rawJ2(:)-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);
dSensP = sensP_Zero - sensP_Cur;
J_Len = c - dSensP;

Q_jointJ2(:) = (acosd(((J_Len.^2)-(a^2)-(b^2))/(-2*a*b))-acosd(((c^2)-(a^2)-(b^2))/(-2*a*b)));
