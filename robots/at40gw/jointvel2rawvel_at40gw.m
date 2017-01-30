function [ qdraw ] = jointvel2rawvel_at40gw( robot, q, qd )
% JOINTVEL2RAWVEL_AT40GW Converts joint velocities to raw sensor velocities
% for the AT40GW

%{
jointvel2rawvel_at40gw.m
Julian Leland, MIT Media Lab,2017-01-24

This function converts joint velocities (deg/sec or mm/sec) to sensor
velocities (counts/sec or V/sec).

INPUTS:
  qd - [j1, j2, j3, j4], j1..3 in deg/sec, j4 in mm/sec
OUTPUTS:
  qdraw - [counts/sec, V/sec, V/sec, V/sec]

NOTES:
    J1: 737,280 counts/revolution
    J3: -4.4707 V/deg
    J4: 3350 mm/20 volts, or .006 V/mm
    TODO: Need joint 2
    TODO: Need to verify joint 3 is correct
%}
    
    qdraw = zeros(size(qd));
    qdraw(:,1) = qd(:,1) .* 737280/360;
    %qdraw(:,3) = qd(:,3) ./ -4.4707;
    
    qdraw(:,3) = djoint2draw_j3(robot, q(:,3), qd(:,3));
    
    qdraw(:,4) = qd(:,4) .* 0.006;
    
end