function [ Q ] = ikine_at40gw( X, q0, used )
%IKINE_AT40GW Iterative inverse kinematics solver for DCP

%{
ikine_at40gw.m
Julian Leland, MIT Media Lab, 2017-01-24
Originally written by Veevee Cai, MIT Media Lab, 2016

This function implements an iterative inverse kinematics solver for the
AT40GW portion of the DCPv2 It is based on Peter Corke's ikine function,
but has been modified to use the custom DCP Jacobian and forward kinematics
algorithms.

INPUTS:
  X - Nx3 matrix of cartesian coordinates of the end-effector of AT40GW
  q0 - an initial guess for the first point to invert (zeros default)
  used - the joints to use, [1 0 1 1] default as we usually fix joint 2

OUTPUTS:
  Q: Nx4 matrix of joint-space coordinates for DCP corresponding to X.
%}

alpha = 0.1;
tol = 1e-6;
ilimit = 1000;

npoints = size(X,1);

Q = zeros(npoints,4);

if nargin < 2
    q0 = zeros(1,4);
end

if nargin < 3
    used = [1 0 1 1];
end

q = q0;
used = logical(used);
used_ind = find(used);
for i = 1:npoints
    %i
    % New end point
    p = X(i,:);
    
    while true
        % Compute errors
        pe = joint2cart_at40gw(q);
        v = p - pe;
        
        % Adjust step size (incorporate after getting fixed step size
        % working)
        % Logic from RVC toolbox, if error decreased, take a bigger step,
        % otherwise, take smaller steps
%         if norm(v) < prev_nv
%             % error reduced,
%             % let's save current state of solution and rack up the step size
%             alpha = alpha * (2.0^(1.0/8));
%         else
%             % restore to last good solution and reduce step size
%             q = prev_q;
%             v = prev_v;
%             alpha = alpha * 0.5;
%         end
        
        % Compute the jacobian
        J = jacobian_at40gw(q(1),q(2),q(3),q(4));
        
        dq_used = J(:,used)\v'; % do we need pinv?
        %dq_used = pinv(J(:,used))*v';
        
        dq = zeros(1,4);
        dq(used_ind) = dq_used;
        
        dq = alpha * dq;
        
        q = q + dq;
        
        %norm(v)
        if norm(v) < tol
            Q(i,:) = q;
            break;
        end
    end
    
end

end

