% symraw2joint
% Attempt at building symbolic raw2joint so we can create matrices

%% Setup
syms qr1 qr2 qr3 qr4

r2j = sym('r2j',[1,4]);


%% Joint 1: Encoder counts to degrees
% Old version - used before analog encoder installed
%counts2rev = 1/robot.Joint(1).PosSensParams.CountsRev;
%r2j(1) = qr1 .* counts2rev .* 360;

jointV_Zero = robot.Joint(1).PosSensParams.ZeroVoltage; % Sensor voltage at zero position
jointV_Max = robot.Joint(1).PosLim.Max; % Maximum sensor voltage
jointV_Min = robot.Joint(1).PosLim.Min; % Minimum sensor voltage
jointq_Max = robot.Joint(1).PosSensParams.PosqLim; % Joint position at jointV_Max
jointq_Min = robot.Joint(1).PosSensParams.NegqLim; % Joint position at jointV_Min
r2j(1) = ((qr1-jointV_Min).*(jointq_Max - jointq_Min))./(jointV_Max - jointV_Min) + jointq_Min;

%% Joint 2: Voltages to degrees
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
sensP_Cur = ((qr2-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);
dSensP = sensP_Zero - sensP_Cur;
J_Len = c - dSensP;

r2j(2) = ((acos(((J_Len.^2)-(a^2)-(b^2))/(-2*a*b))*(180/pi))-(acos(((c^2)-(a^2)-(b^2))/(-2*a*b))*(180/pi)));

%% Joint 3: Voltages to degrees
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
sensP_Cur = ((qr3-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);
dSensP = sensP_Zero - sensP_Cur;
J_Len = c - dSensP;
r2j(3) = ((acos(((c^2)-(a^2)-(b^2))/(-2*a*b))*(180/pi))-(acos(((J_Len.^2)-(a^2)-(b^2))/(-2*a*b))*(180/pi)));

%% Joint 4: Voltage to mm
a = robot.Joint(4).PosSensParams.a_len; % Distance between joint axis and cylinder base axis
b = robot.Joint(4).PosSensParams.b_len; % Distance between joint axis and cylinder attachment point axis
c = robot.Joint(4).PosSensParams.c_len; % Length of cylinder at J_Angle = 0
jointV_Zero = robot.Joint(4).PosSensParams.ZeroVoltage; % Sensor voltage at zero position
jointV_Max = robot.Joint(4).PosLim.Max; % Maximum sensor voltage
jointV_Min = robot.Joint(4).PosLim.Min; % Minimum sensor voltage
sensLen = robot.Joint(4).PosSensParams.SensLen; % Length of joint sensor (allows us to figure out where we are along joint)

% We need to make the orientation of our max/min sensor values match the
% orientation of the joint we're working on.
sensVMax = 10*sign(jointV_Max); 
sensVMin = 10*sign(jointV_Min);

% Find position along sensor of joint zero, relative to center
sensP_Zero = ((jointV_Zero-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);

% At current voltage, how far away from jointP_Zero are we?
sensP_Cur = ((qr4-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);
dSensP = sensP_Zero - sensP_Cur;
J_Len = c - dSensP;
r2j(4) = J_Len;

%% Create MATLAB function for raw2joint
f = matlabFunction(r2j,'File','r2jfcn_at40gw');

% This works!

%% Test r2j function
% Note: For these tests to be vaild, they require a trajectory that's been
% generated with our known-working method from Summer 2016.
qjoint_sym = r2jfcn(traj.qraw.data(:,1),traj.qraw.data(:,2),traj.qraw.data(:,3),traj.qraw.data(:,4));
figure(1);
for n = 1:4
    subplot(4,1,n);
    plot(qjoint_sym(:,n)-traj.q.Data(:,n));
    title(['Joint ',num2str(n)]);
end
suptitle('r2jfcn(qraw) output minus q');
%% Take derivative of matrix and create MATLAB function
dr2j = [diff(r2j(1),qr1), 0, 0, 0;
        0, diff(r2j(2),qr2), 0, 0;
        0, 0, diff(r2j(3),qr3), 0;
        0, 0, 0, diff(r2j(4),qr4)];

g = matlabFunction(dr2j,'File','dr2djfcn_at40gw');

%% Test dr2dj function
% This function works similarly to the Jacobian. For a given
qdjoint_sym = zeros(length(traj.qraw.Data),4);
for n = 1:length(traj.qraw.Data)
    qdjoint_sym(n,:) = (dr2djfcn(traj.qraw.Data(n,2),traj.qraw.Data(n,3))*traj.qdraw.Data(n,:)')';
end

figure(2);
for n = 1:4
    subplot(4,1,n);
    plotyy(traj.qd.Time,qdjoint_sym(:,n)-traj.qd.Data(:,n),traj.qd.time,traj.qd.Data(:,n));
    title(['Joint ',num2str(n)]);
end
suptitle('dr2djfcn(qraw) output minus qdraw');

% Alternative code for looking at a single joint's output:
%{
n = 3;
plot(traj.qd.time,traj.qd.Data(:,n),traj.qd.time,qdjoint_sym(:,n));
%}

%% Invert matrix
% This doesn't work. J1, J3 and J4 all work fine, but J2 does something
% strange.
%{
j2r = [finverse(r2j(1),qr1),finverse(r2j(2),qr2),finverse(r2j(3),qr3),finverse(r2j(4),qr4)];

h = matlabFunction(j2r,'File','j2rfcn');
%}
