function X = joint2cart_at40gw( Q )
%JOINT2CART_AT40GW Converts  joint-space positions to Cartesian positions
%of the AT40GW endpoint using forward kinematics.

%{
joint2cart_at40gw.m
Julian Leland, MIT Media Lab, 2017-01-24
Originally written by Veevee Cai, MIT Media Lab, 2016

This function converts joint-space positions to a Cartesian position of the
AT40GW endpoint (end of KUKA arm) using forward kinematics. The KUKA arm is
treated as a fixed offset here - deviations from its starting position
(programmed into forward and inverse kinematics) are not included here.

INPUTS:
  Q - Nx4 matrix, column per joint, must be in degrees and mm
OUTPUTS:
  X - Nx3 matrix, [X, Y, Z] cartesian positions of the end-effector of
  the boom arm
%}

X = zeros(size(Q,1), 3);
for i = 1:size(Q,1)
    fk = fkine_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    p = fk * [1; 0; 0; 0];
    X(i, :) = p(2:4)';
end

end

