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
colors = get(gca,'ColorOrder'); % Set up colormap

%% Open file
fileID = fopen('color wheel_2.txt'); % Type pathname to file here!
%fileID = fopen('DCP_test_8_FIXED.txt'); % Type pathname to file here!

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

%% Check for duplicates and delete if necessary
for n = 1:size(waypts,1)
    [~,waypts_idx,~] = unique(waypts{n,1}(:,1),'stable');
    waypts_temp = waypts{n,1}(waypts_idx,:);
    waypts{n,1} = waypts_temp;
end

%% Plot each segment's parts
if vizflag  
    j = 1;
    h = figure(1);
    grid on
    view([60,20]);
    axis square
    axis vis3d
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title({'Imported Waypoints and Segments'})
    hold on
    for n = 1:size(waypts,1)
        for m = 1:size(waypts{n},1)
            scatter3(waypts{n}(m,2),waypts{n}(m,3),waypts{n}(m,4),20,[colors(j,:)]);
            drawnow;
        end
        plot3(waypts{n}(:,2),waypts{n}(:,3),waypts{n}(:,4),'Color',[colors(j,:)]);
        j = j+1;
        if j >= length(colors)
            j = 1;
        end
    end
end

%% Clear unneeded variables
disp('Waypoints imported successfully!');
curvars = {curvars{:},'waypts'};
clearvars('-except', curvars{:});
clear curvars;
close([1]);

%%
%{
for n=1:size(waypts,1)
    waypts{n,1}(:,2:3) = [waypts{n,1}(:,3),waypts{n,1}(:,2)]
end
%}