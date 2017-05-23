%{
init.m
Julian Leland, MIT Media Lab, 2016-11-18

Startup function for dcpctrl_v2 library. Run once to initialize all
directories
%}

%% Initialize path
addpath(genpath([pwd '/apps']));
addpath(genpath([pwd '/robots']));
addpath(genpath([pwd '/lib']));
addpath([pwd '/data']);
addpath(genpath([pwd '/util']));
addpath([pwd '/template']);

% Remove any directories from path that you don't want searched
rmpath([pwd '/apps/examples/slxMDCPFcnDemo']);
%% Initialize apps (might not be necessary...)

%% Initialize AT40GW functions
%if exist('robot')~=1
disp('Loading AT40GW configuration');
[robot, ts] = config_dcpv2;
%end