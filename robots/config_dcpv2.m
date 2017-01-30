function robot = config_dcpv2
% CONFIG_DCPV2 Set up configuration for DCP v.2
% OUTPUT:
%   robot - structure containing DCP configuration values
% INPUT: None.
%{
    robot = config_dcpv2
    Julian Leland, MIT Media Lab, 2016-11-18
    
    This script replaces config_at40gw from the mdcp control library. It
    performs the same functions, but sets up more workspace variables so
    that they can be accessed from Simulink.
%}

%% Control Parameters
% Proportional gains: Best practice is to set in following ranges:
% - J1: 50 to 5000
% - J2 thru J4: 45 to 175
% -- Note: J4 gain needs to be inverted - J4 has its PWM mapping backwards
Kp = struct('J1',0.3,'J2',0,'J3',0.6,'J4',0.5);
Kd = struct('J1',0,'J2',0,'J3',0.3,'J4',0);
Ki = struct('J1',0,'J2',0,'J3',0,'J4',0);
Kv = struct('J1',0,'J2',0,'J3',0.5,'J4',0);

%% DH Parameters
% Create symbolic variables for joint angles/positions
syms Theta_1 Theta_2 Theta_3 D_4;

Base_DH = struct('a',0,'alph',0,'d',0,'the',0);
J1_DH = struct('a',609.50,'alph',-(pi/2),'d',711.30,'the', 0 + (Theta_1*(pi/180)));
J2_DH = struct('a',2533.65,'alph',0,'d',0,'the',pi + (Theta_2*(pi/180)));
InnerLink_DH = struct('a',171.94,'alph',0,'d',0,'the',(pi/4) + -(Theta_2*(pi/180)));
J3_DH = struct('a',59.02,'alph',-(pi/2),'d',-329.41,'the',(pi/4) + (Theta_3*(pi/180)));
J4_DH = struct('a',0,'alph',0,'d',4633.10 + D_4,'the',0);

% Some of our axes are inverted: negative PWM duty cycles correspond to
% positive sensor changes. GainInv accounts for this by multiplying the
% gains set above by 1 (normal) or -1 (inverted)
GainInv = struct('J1',1,'J2',1,'J3',-1,'J4',-1);

% moveThresh is different for J1 than other axes - it is defined in counts
% instead of volts.
moveThresh = struct('J1',500,'J2',0.04,'J3',0.04,'J4',0.04);
pctMaxV = struct('J1',1,'J2',1,'J3',1,'J4',1);

%% Machine geometric, controller & valve parameters
% Position limits
J1PosLim = struct('Max',1000000,'Min',-1000000);
% J1PosLim = struct('Max',10,'Min',0); % Values for analog joint output sensor. This will need to be adjusted.
% NOTE: J2PosLim has an opposite sense from the other joints. This is
% because of the way that the J2 sensor is oriented relative to the
% positive angle direction (as defined by the DH parameters), which is
% different from the other axes. Don't change this!
J2PosLim = struct('Max',-9.508,'Min',8.075);
J3PosLim = struct('Max',9.912,'Min',-9.337);
J4PosLim = struct('Max',9.488,'Min',-9.925);

% PWM signal limits - generated from last known working parameters for
% electric drive from config_at40gw.m. These values are in % duty cycle.
J1PWMLim = struct('Max',155500/160000,'Min',5000/160000,'DBUpper',89000/160000,'DBLower',69000/160000);
J2PWMLim = struct('Max',155500/160000,'Min',5000/160000,'DBUpper',93000/160000,'DBLower',70000/160000);
J3PWMLim = struct('Max',155500/160000,'Min',5000/160000,'DBUpper',91000/160000,'DBLower',72000/160000);
% Anything less than 43000 all gives full speed
J4PWMLim = struct('Max',155500/160000,'Min',43000/160000,'DBUpper',91000/160000,'DBLower',69000/160000);

PWMLim = struct('J1',J1PWMLim,'J2',J2PWMLim,'J3',J3PWMLim,'J4',J4PWMLim);

% Voltage/position conversion factors
J1VtoAngVals = struct('ZeroVoltage',0,'SensLen','','a_len','','b_len','','c_len','','CountsRev',737280); % This is a direct revolute joint
J2VtoAngVals = struct('ZeroVoltage',6.79,'SensLen',400,'a_len',283.978,'b_len',754.053,'c_len',666.334,'CountsRev',''); % This is a revolute joint, with piston in a linkage.
J3VtoAngVals = struct('ZeroVoltage',-3.48,'SensLen',600,'a_len',1163.725,'b_len',387.085,'c_len',1082.457,'CountsRev',''); % This is a revolute joint, with piston in a linkage.
J4VtoAngVals = struct('ZeroVoltage',-9.925,'SensLen',3350,'a_len','','b_len','','c_len',0,'CountsRev',''); % This is a prismatic joint, with piston NOT in a linkage - direct drive.

% Velocity mapping (note: these will be removed in the future, just using
% for now to accomodate legacy code)
J1VelMap = struct('PosKnee',17200,'NegKnee',-17200,'PosVelLim',384.2510,'NegVelLim',-1.2675e3); % VelLim values copied from old calculations - believed to be wrong.
J2VelMap = struct('PosKnee',1000,'NegKnee',-1000,'PosVelLim',1000,'NegVelLim',-1000); % Dummy values - never tested
J3VelMap = struct('PosKnee',1.39,'NegKnee',-1.39,'PosVelLim',.1609,'NegVelLim',-.1292);
J4VelMap = struct('PosKnee',.91,'NegKnee',-.77,'PosVelLim',0,'NegVelLim',0);


%% Construct starting structure
robot = struct('Name', 'DCP_V2');
robot.PWMZero = 0.5;
robot.HomeRawPos = [0, 0, -2.5664, -7.7892];
robot.Joint(1) = struct('Name','Joint 1','EncType','Rotary','Kp',Kp.J1,'Kd',Kd.J1,'Ki',Ki.J1,'Kv',Kv.J1,'GainInv',GainInv.J1,'moveThresh',moveThresh.J1,'pctMaxV',pctMaxV.J1,'PosLim',J1PosLim,'PWMLim',J1PWMLim,'PWM_zero',0.5,'PosSensParams',J1VtoAngVals,'VelMap',J1VelMap,'PWMDBUp',J1PWMLim.DBUpper,'PWMDBLo',J1PWMLim.DBLower,'PWMDBMax',J1PWMLim.Max,'PWMDBMin',J1PWMLim.Min);
robot.Joint(2) = struct('Name','Joint 2','EncType','Linear','Kp',Kp.J2,'Kd',Kd.J2,'Ki',Ki.J2,'Kv',Kv.J2,'GainInv',GainInv.J2,'moveThresh',moveThresh.J2,'pctMaxV',pctMaxV.J2,'PosLim',J2PosLim,'PWMLim',J2PWMLim,'PWM_zero',0.5,'PosSensParams',J2VtoAngVals,'VelMap',J2VelMap,'PWMDBUp',J2PWMLim.DBUpper,'PWMDBLo',J2PWMLim.DBLower,'PWMDBMax',J2PWMLim.Max,'PWMDBMin',J2PWMLim.Min);
robot.Joint(3) = struct('Name','Joint 3','EncType','Linear','Kp',Kp.J3,'Kd',Kd.J3,'Ki',Ki.J3,'Kv',Kv.J3,'GainInv',GainInv.J3,'moveThresh',moveThresh.J3,'pctMaxV',pctMaxV.J3,'PosLim',J3PosLim,'PWMLim',J3PWMLim,'PWM_zero',0.5,'PosSensParams',J3VtoAngVals,'VelMap',J3VelMap,'PWMDBUp',J3PWMLim.DBUpper,'PWMDBLo',J3PWMLim.DBLower,'PWMDBMax',J3PWMLim.Max,'PWMDBMin',J3PWMLim.Min);
robot.Joint(4) = struct('Name','Joint 4','EncType','Linear','Kp',Kp.J4,'Kd',Kd.J4,'Ki',Ki.J4,'Kv',Kv.J4,'GainInv',GainInv.J4,'moveThresh',moveThresh.J4,'pctMaxV',pctMaxV.J4,'PosLim',J4PosLim,'PWMLim',J4PWMLim,'PWM_zero',0.5,'PosSensParams',J4VtoAngVals,'VelMap',J4VelMap,'PWMDBUp',J4PWMLim.DBUpper,'PWMDBLo',J4PWMLim.DBLower,'PWMDBMax',J4PWMLim.Max,'PWMDBMin',J4PWMLim.Min);

%% Generate 
end