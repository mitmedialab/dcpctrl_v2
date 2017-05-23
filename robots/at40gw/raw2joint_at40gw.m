function Q_joint = raw2joint_at40gw(robot, Q_raw)
% RAW2JOINT_AT40GW Converts raw sensor readings to joint-space positions of
% the AT40GW, where revolute joints 1...3 are in degrees and prismatic
% joint 4 is in mm.

%{
raw2joint_at40gw.m
Julian Leland, MIT Media Lab, 2017-05-23

This function converts raw sensor readings to joint positions for the
AT40GW (joints 1 through 4 of the DCP).

This is a revised version of the previous raw2joint function, which uses
the 

INPUT:
  robot - robot definition
  Q_raw - Nx4 matrix of raw joint values (joint 1 is encoder counts, 2..4
      are in voltages
OUTPUT:
  Q_joint - Nx4 matrix of joint-space values in degrees and mm (use
  deg2rad to convert to radians)
%}

% Set up outputs and inputs for function
Q_joint = zeros(size(Q_raw)); % Store Q_joint data

Q_rawJ2 = Q_raw(:,2);
Q_rawJ3 = Q_raw(:,3);
Q_rawJ4 = Q_raw(:,4);


% Joint 1: Encoder counts to degrees
% NOTE: This will need to be revised once the J1 absolute sensor is
% installed.
counts2rev = 1/robot.Joint(1).PosSensParams.CountsRev;

Q_joint(:,1) = Q_raw(:,1) .* counts2rev .* 360;

% Joint 2: Voltages to degrees
Q_joint(:,2) = raw2jointJ2Abs_at40gw(robot, Q_rawJ2);

% Joint 3: Voltages to degrees
Q_joint(:,3) = raw2jointJ3Abs_at40gw(robot, Q_rawJ3);

% Joint 4: Voltage to mm
Q_joint(:,4) = raw2jointJ4Abs_at40gw(robot, Q_rawJ4);

end