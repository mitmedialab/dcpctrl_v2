% symjoint2raw
% Attempt at building symbolic joint2raw so we can easily differentiate,
% run faster, etc.

%% Setup
syms q1 q2 q3 q4

j2r = sym('j2r',[1,4]);

%% Joint 1: Degrees to encoder counts (measured relative to whatever start position was - we're still using an incremental encoder)

j2r(1) = q1 ./ 360 .* robot.Joint(1).PosSensParams.CountsRev;


%% Joint 2: Degrees to volts

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

Zero_Angle = acosd(((c^2)-(a^2)-(b^2))/(-2*a*b));
J_Len = sqrt(a^2 + b^2 - 2*a*b*cos((q2+Zero_Angle)*(pi/180)));
J_dLen = J_Len-c; % Calculate change in distance from zero length of joint
J_dVolts = mapRange(J_dLen,(-sensLen/2),(sensLen/2),sensVMin,sensVMax);
%J_dVolts =
%((J_dLen-(-sensLen/2))*(sensVMax-sensVMin))/((sensLen/2)-(-sensLen/2))+
%sensVMin; % Use this line instead if we don't want to use mapRange.

j2r(2) = jointV_Zero + J_dVolts;

%% Joint 3: Degrees to volts

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

Zero_Angle = acosd(((c^2)-(a^2)-(b^2))/(-2*a*b));
J_Len = sqrt(a^2 + b^2 - 2*a*b*cos((q3-Zero_Angle)*(pi/180))); % Joint 3 is treated differently than Joint 2 because of inverted relationship between triangle angle & "positive" angle as defined by DH parameters.
J_dLen = J_Len-c; % Calculate change in distance from zero length of joint
J_dVolts = mapRange(J_dLen,(-sensLen/2),(sensLen/2),sensVMin,sensVMax);
j2r(3) = jointV_Zero + J_dVolts;

%% Joint 4: Millimeters to volts

c = robot.Joint(4).PosSensParams.c_len; % Length of cylinder at J_Angle = 0
jointV_Zero = robot.Joint(4).PosSensParams.ZeroVoltage; % Sensor voltage at zero position
jointV_Max = robot.Joint(4).PosLim.Max; % Maximum sensor voltage
jointV_Min = robot.Joint(4).PosLim.Min; % Minimum sensor voltage
sensLen = robot.Joint(4).PosSensParams.SensLen; % Length of joint sensor (allows us to figure out where we are along joint)

% We need to make the orientation of our max/min sensor values match the
% orientation of the joint we're working on.
sensVMax = 10*sign(jointV_Max); 
sensVMin = 10*sign(jointV_Min);

J_dLen = q4-c; % Calculate change in distance from zero length of joint
J_dVolts = mapRange(J_dLen,(-sensLen/2),(sensLen/2),sensVMin,sensVMax);
j2r(4) = jointV_Zero + J_dVolts;

%% Create MATLAB function for joint2raw
f = matlabFunction(j2r,'File','j2rfcn_at40gw');

%% Take derivative to find djoint2draw matrix and create MATLAB function
% Note - we can't do any rounding of j2r(1) if we do this. Need to correct
% to counts elsewhere
dj2dr = [diff(j2r(1),q1), 0, 0, 0;
        0, diff(j2r(2),q2), 0, 0;
        0, 0, diff(j2r(3),q3), 0;
        0, 0, 0, diff(j2r(4),q4)];

g = matlabFunction(dj2dr,'File','dj2drfcn_at40gw');

%% Test derivative
% This function works similarly to the Jacobian.
qdraw_sym = zeros(length(traj.qd.Data),4);
for n = 1:length(traj.qd.Data)
    qdraw_sym(n,:) = (dj2drfcn(traj.q.Data(n,2),traj.q.Data(n,3))*traj.qd.Data(n,:)')';
end

% Example code for looking at differences in outputs
n = 3; plot(qdraw_sym(:,n)-traj.qdraw.Data(:,n));
