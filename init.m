%{
init.m
Julian Leland, MIT Media Lab, 2016-11-18

Startup function for dcpctrl_v2 library. Run once to initialize all
directories
%}

%% Initialize path
addpath(genpath('apps'));
addpath(genpath('robots'));
addpath('lib');
addpath('data');
addpath('util')
addpath('template');

%% Initialize apps (might not be necessary...)

%% Initialize AT40GW functions
%if exist('robot')~=1
disp('Loading AT40GW configuration');
robot = config_dcpv2;
%end