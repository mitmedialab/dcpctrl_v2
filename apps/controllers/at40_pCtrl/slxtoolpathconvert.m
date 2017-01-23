%{
slxtoolpathconvert.m
Julian Leland
2017-01-11

This function takes toolpaths generated using dcpctrl_v1 techniques and
transforms them into a form usable under dcpctrl_v2.

%}

%% Set up temp variables for timeseries
qraw_temp = [];
qdraw_temp = [];

%% Collect trajs and measure total trajectory length
trajlen = 0;
for n = 1:length(qrawtrajs)
    trajlen = trajlen + sum(length(qrawtrajs{n}));
    qraw_temp = [qraw_temp;qrawtrajs{n}];
    qdraw_temp = [qdraw_temp;qdrawtrajs{n}];    
end

%% Create time vector
t_temp = linspace(0,trajlen*dt,trajlen);

%% Convert into timeseries
rawtraj = timeseries([qraw_temp,qdraw_temp],t_temp);
