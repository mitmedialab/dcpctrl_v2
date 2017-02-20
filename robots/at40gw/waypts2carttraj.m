function [carttraj] = waypts2carttraj( waypts, dt, spd, tacc )
%WAYPTS2CARTTRAJ Convert waypoints (x,y,z,h) to task-space (Cartesian)
%trajectory

%{
waypts2carttraj.m
Julian Leland, MIT Media Lab, 2017-02-17

This function takes a series of ordered waypoints in the format (x,y,z)
and converts them to a time-parametrized, task-space (Cartesian) trajectory
of the form (x(t),y(t),z(t)).

INPUTS:
- waypts: Cell array of form:

    {[xa1,ya1,za1;...xan,yan,zan],[h1],[h2]...[hn]}. 

-- The first [x,y,z] matrix is assumed to containt Cartesian position 
coordinates. This must be present, and is used to define the length of the
trajectory.
-- Subsequent [h] vectors/matrices are used to generate empty matrices
which can have other types of trajectories (tool, KUKA). The number of rows
of these elements is ignored - the number of columns is the only
information used.
- dt: Timestep for trajectory, s.
- spd: Cartesian velocity for trajectory, distance units/s.
- tacc: Cartesian acceleration time, distance units/s^2.

OUTPUTS:
- carttraj: Cell array of form:
    
    {[xa(t), ya(t), za(t)],[h1(t)],[h2(t)],...[hn(t)]}

%}
%% Detect size of waypoints array and data check

if(size(waypts{1},2) ~= 3);
    disp('Incorrectly sized Cartesian waypoints vector! Exiting trajectory generation.');
    carttraj = {};
    return;
end

numh = size(waypts,2)-1;

%% Generate toolapth and time vector
disp('Generating trajectory. Please stand by...');
[xtraj, xdtraj] = mstraj2(waypts{1}(2:end,1:3),spd,[],waypts{1}(1,1:3),dt,tacc);
ts = (0:(size(xtraj,1)-1)) .* dt;

%% Generate appropriately sized zero vectors for other trajectory components
htraj = cell(1,numh);
for n = 1:numh
    htraj{n} = zeros(length(ts),size(waypts{n+1},2));
end
    
%% Recombine into carttraj
carttraj = {[xtraj, xdtraj, ts'],htraj{:}};

end

