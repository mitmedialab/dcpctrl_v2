function [carttraj] = dwelltraj( waypts, dt, dwelltime )
%DWELLTRAJ Create zero-movement trajectory for defined amount of time

%{
dwelltraj.m
Julian Leland, MIT Media Lab, 2017-07-31

EXPLANATION

INPUTS:

OUTPUTS:

%}

%% Detect size of waypoints array and data check

if(size(waypts{1},2) ~= 3);
    disp('Incorrectly sized Cartesian waypoints vector! Exiting trajectory generation.');
    carttraj = {};
    return;
end

numh = size(waypts,2)-1;

%% Generate time vector and position/velocity vectors
disp('Generating dwell trajectory. Please stand by...');

ts = 0:dt:dwelltime;

xtraj = repmat(waypts{1}(end,:),length(ts),1);
xdtraj = repmat([0,0,0],length(ts),1);

%% Generate appropriately sized zero vectors for other trajectory components
htraj = cell(1,numh);
for n = 1:numh
    htraj{n} = ones(length(ts),size(waypts{n+1},2)).*waypts{n+1}; % Multiplying by waypts{n+1} preserves values that are passed in.
end
    
%% Recombine into carttraj
carttraj = {ts',[xtraj, xdtraj],htraj{:}};
disp('Dwell trajectory completed!');

end