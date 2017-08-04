% RHINOWAYPTIMPORT Import toolpaths generated in Rhino using Barrak's
% generator.

%{
rhinowayptimport.m
Julian Leland, MIT Media Lab
2017-08-01

This script imports toolpaths created using Barrak Darweesh's
Rhino/Grasshopper toolpath generation system. This function expects
toolpaths in text files, in the following format

t,x,y,z,h1,h2,h3,h4,h5,h6;...
t,x,y,z,h1,h2,h3,h4,h5,h6;...

Segments are delimited by newline characters. Within segments, waypoint
sets - consisting of a timestamp, XYZ position, and 6 tool command values -
are delimited by semicolons. Within waypoint sets, individual values are
delimited with commas.

%}

%% Get list of all variables currently in workspace
curvars = who; % Get current variables

%% Setup
vizflag = 1;

%% Open file
fileID = fopen('DCP_test_7.txt'); % Type pathname to file here!

%% Separate based on newline delimiter
segs = textscan(fileID,'%s');
segs_temp = segs{1};

%% Re-separate based on semicolon delimiter
waypts = {};
for n = 1:length(segs_temp)
    waypt_temp = textscan(segs_temp{n},'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f','Delimiter',';');
    waypt_mat = [waypt_temp{:}];
    waypts{n,1} = waypt_mat;
end

%% Plot each segment's parts
if vizflag  
    h = figure(1);
    grid on
    view(50,50)
    hold on
    for n = 1:size(waypts,1)
        for m = 1:size(waypts{n},1)
            scatter3(waypts{n}(m,2),waypts{n}(m,3),waypts{n}(m,4));
            drawnow;
            %pause(0.05)
        end
    end
end

%% Clear unneeded variables
disp('Waypoints imported successfully!');
curvars = {curvars{:},'waypts'};
clearvars('-except', curvars{:});
clear curvars;
close([1]);