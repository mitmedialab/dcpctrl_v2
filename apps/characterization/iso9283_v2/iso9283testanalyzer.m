% ISO9283TESTANALYZER.M
% Performs calculations specified in ISO 9283 standard. Assumes that data
% has been broken up into segments segts using iso9283testsegmenter.m.
% Updated version of old iso2983analyzer.m

%{
    Julian Leland, MIT Media Lab, 2016-08-08

    Notes from old code

    Input from new converter looks like:
    [index,x,y,z,timestamp,waypoint_number,run_number]; 

    Old input is divided up in a cell. Each cell corresponds to a segment,
    and internal segmentation is performed in the the analysis code.
    We first examine the first run. We average all positions considered
    "stopped" and use that to define a waypoint location. We then average all
    measurements at a given position and use that to determine our "stopped"
    position for that run

%}

%% Get list of all variables currently in workspace
curvars = who; % Get current variables

%% TEMPORARY: Convert data to mm
% This is only for initial ISO 9283 test that JL conducted 2017-08-07.
% After converting to mm, filters for segmenter would need to be readjusted
% - so JL moved conversion to here.

data_out(:,2:4) = data_out(:,2:4).*10;


%% Figure out how many segments and waypoints
numWaypts = max(data_out(:,6)); % Get number of waypoints in this test
for n = size(data_out,1):-1:1 % Count backwards for end
    if data_out(n,6) == numWaypts % Find last occurence of last waypoint
        numSegts = data_out(n,7);
        break;
    end
end

%% Divide data up into segments
segts = {}; % Empty cell array to store segments
for n = 1:numSegts
    segts{n} = data_out(find(data_out(:,7) == n),:); % Get only the data for the current segment
end

%% For every segment, get mean value of all measurements at each waypoint
segt_meanPoses = {}; 
for n = 1:numSegts
    cur_Segt = data_out(find(data_out(:,7) == n),:); % Get only the data for the current segment
    for m = 1:numWaypts
        cur_Waypt = cur_Segt(find(cur_Segt(:,6) == m),:);
        segt_meanPoses{n,1}(m,:) = [mean(cur_Waypt(:,2)),mean(cur_Waypt(:,3)),mean(cur_Waypt(:,4))];
    end
end

%% Plot average poses at all locations - sanity check
figure(1)
hold on
grid on
view(50,50)
for n = 1:length(segt_meanPoses)
    scatter3(segt_meanPoses{n,1}(:,1),segt_meanPoses{n,1}(:,2),segt_meanPoses{n,1}(:,3));
    drawnow;
    pause(0.5)
end

%% Repeatability
% Calculate mean X, Y, Z values for start and end positions - x_bar, y_bar, z_bar in
% ISO 9283

% Collect mean positions at each waypoint into vectors
mean_posVect = repmat({[]},numWaypts,1);
for n = 1:size(segt_meanPoses,1)
    for m = 1:numWaypts
        mean_posVect{m} = [mean_posVect{m};segt_meanPoses{n}(m,:)];
    end
end

% Find mean of mean positions
mean_pos = zeros(numWaypts,3);
for n = 1:length(mean_posVect)
    mean_pos(n,:) = mean(mean_posVect{n,1});
end
        
% Sanity check - plot mean waypoints against all waypoints
%{
figure(2)
hold on
grid on
view(50,50)
scatter3(mean_pos(:,1),mean_pos(:,2),mean_pos(:,3),2,'r')
for n = 1:length(segt_meanPoses)
    scatter3(segt_meanPoses{n,1}(:,1),segt_meanPoses{n,1}(:,2),segt_meanPoses{n,1}(:,3),2,'k');
end
%}

% Calculate mean Cartesian error
l_j = zeros(length(mean_pos),1); % Vector to store mean Cartesian errors
temp_err = [];
for n = 1:numSegts
    for m = 1:numWaypts
        temp_err = sqrt((mean_pos(m,1)-segt_meanPoses{n,1}(m,1))^2 + (mean_pos(m,2)-segt_meanPoses{n,1}(m,2))^2 + (mean_pos(m,3)-segt_meanPoses{n,1}(m,3))^2);
        l_j(m,n) = temp_err;
    end
end

l_jbar = sum(l_j,2)/length(l_j); % This is the mean Cartesian error

% Calculate std deviation (corrected sample std dev, per ISO 9283)
l_j_ssq = bsxfun(@minus,l_j,l_jbar).^2;
si = sqrt(sum(l_j_ssq,2)/(length(l_j_ssq)-1));
posrep = l_jbar + 3*si;


%% Plot full run data, with spheres representing repeatability ranges around endpoints

j = 1; % Index to start colormap at
figure;

colors = get(gca,'ColorOrder');
posVect1 = [ 0.05 0.3 0.45 0.65];
fig1 = subplot('Position',posVect1)
hold on;

% Plot the 3D trajectory
for n = 1:length(segts)
    plot3(segts{1,n}(:,2),segts{1,n}(:,3),segts{1,n}(:,4),'Color',[colors(j,:)]);
    j = j+1;
    if j == 7
        j = 1;
    end
end

% Plot a repeatability sphere for each waypoint
txtoffst = 50; % Lateral offset for text labels

for n = 1:length(posrep)
    phi=linspace(0,pi,30);
    theta=linspace(0,2*pi,40);
    [phi,theta]=meshgrid(phi,theta);

    xs=posrep(n,1)*sin(phi).*cos(theta) + mean_pos(n,1);
    ys=posrep(n,1)*sin(phi).*sin(theta) + mean_pos(n,2);
    zs=posrep(n,1)*cos(phi) + mean_pos(n,3);

    plot3(xs,ys,zs,':','Color','red');
    text(mean_pos(n,1)+txtoffst,mean_pos(n,2)+txtoffst,mean_pos(n,3),['Waypoint ',num2str(n)]);
end

grid on
axis image
axis equal
xlabel('X, mm');
ylabel('Y, mm');
zlabel('Z, mm');
title({'3D Position';'Red spheres = repeatability boundaries';''},'FontWeight','Normal');
view(-45,30)

% Plot the 2D plot
j = 1;
posVect2 = [ 0.55 0.3 0.4 0.65];
fig2 = subplot('Position',posVect2);
hold on;
for n = 1:length(segts)
    plot(segts{1,n}(:,5),segts{1,n}(:,2),'Color',colors(1,:));
    plot(segts{1,n}(:,5),segts{1,n}(:,3),'Color',colors(2,:));
    plot(segts{1,n}(:,5),segts{1,n}(:,4),'Color',colors(3,:));
    % Commenting below out so we can plot X, Y, Z data together
%     j = j+1;
%     if j == 7
%         j = 1;
%     end
end
legend('X Position','Y Position','Z Position');
xlabel('Time, s');
ylabel('Measured Position in Direction, mm');
title({'X/Y/Z Position-Per-Axis vs. Time';''},'FontWeight','Normal');

suptitle({'Pose Repeatability: 3D & Position-Per-Axis';'ISO 9283-1998'});

% Plot the run data
posVect3 = [ 0.1 0.05 0.8 0.1];
fig3 = subplot('Position',posVect3,'Visible','off');

% Generate run metrics for each waypoint
results = {};
for n = 1:length(posrep)
    line_n = {['\bf Waypoint ', num2str(n), '\rm - Mean error: ',num2str(l_jbar(n)),' mm | Std. Dev.: ', num2str(si(n)),' mm | Repeatability: ', num2str(posrep(n)),' mm']};
    results = [results;line_n];
end

%results = { ['\bf Start Position\rm - Mean error: ',num2str(l_jsbar),' mm | Std. Dev.: ', num2str(si_s),' mm | Repeatability: ', num2str(posrep_s),' mm'];
 %           ' ';
  %          ['\bf End Position\rm - Mean error: ',num2str(l_jebar),' mm | Std. Dev.: ', num2str(si_e),' mm | Repeatability: ', num2str(posrep_e),' mm']};        

axes(fig3) % sets ax1 to current axes
text(.05,0.6,results)

testname = 'ISO9283CharData\_2017-08-07'; % Put the name of the test here
testspecs = {   ['\bf Test Name:\rm ', testname];
                ' ';
                ['\bf Measurement Unit:\rm HTC Vive VR Tracker | \bf Measurement MPE:\rm Not Defined (accuracy estimated ~±2 mm)']
                ['\bf Measurement Frequency:\rm ',num2str(1/dt),' Hz']};
text(.55,0.6,testspecs)

