function [ QD ] = cartvel2jointvel_at40gw( Q, V, used )
%CARTVEL2JOINTVEL_AT40GW Use inverse jacobian to compute the joint
%velocities

%{
cartvel2jointvel_at40gw.m
Julian Leland, MIT Media Lab, 2017-01-24
Originally written by Veevee Cai, MIT Media Lab, 2016

This function uses the inverse Jacobian to calculate joint-space velocities
from Cartesian velocities for the DCP V2.

INPUTS:
  Q - Nx4 joint positions
  V - Nx3 cartesian velocities
  used - 1x4 joints to be used in computation (default is [1 0 1 1])

OUTPUTS:
 QD - Nx4 joint velocities

%}
if nargin < 3
    used = logical([1 0 1 1]);
end

npoints = size(Q,1);
QD = zeros(size(Q));

% TODO: can probably vectorize this further
used_ind = find(used);
for i = 1:npoints
    q = Q(i,:);
    v = V(i,:);
    J = jacobian_at40gw(q(1),q(2),q(3),q(4));
    
    dq_used = J(:,used)\v';
    dq = zeros(1,4);
    dq(used_ind) = dq_used;
    
    QD(i,:) = dq;
end

end

