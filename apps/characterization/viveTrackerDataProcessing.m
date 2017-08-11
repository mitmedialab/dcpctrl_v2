%% Get list of all variables currently in workspace
curvars = who; % Get current variables

%% Open data
load('data/ISO9283CharData_2017-08-07.mat');

%% Plot segments to see if there's any discrepancy between start points
figure(1);
hold on;
grid on;
axis equal
view(50,50);
plot3(Test6Part1(:,2),Test6Part1(:,3),Test6Part1(:,4),'r');
plot3(Test6Part2(:,2),Test6Part2(:,3),Test6Part2(:,4),'b');

%% Normalize Test 1 time vectors to zero start
% Time is measured in these tests in 0.1 uS
Test6Part1_adj = Test6Part1 - [Test6Part1(1,1) 0 0 0];
Test6Part2_adj = Test6Part2 - [Test6Part2(1,1) 0 0 0];

%% Shift time to seconds
tsamp = 10e-8;
Test6Part1_adj(:,1) = Test6Part1_adj(:,1) .* tsamp;
Test6Part2_adj(:,1) = Test6Part2_adj(:,1) .* tsamp;

figure(2)
hold on
plot(Test6Part1_adj(:,1),Test6Part1_adj(:,2),'b')
plot(Test6Part2_adj(:,1),Test6Part2_adj(:,2),'r')

%% Combine segment sets
%test6Data = Test6Part2 + [Test6Part2(1,:)-Test6Part1(end,:)];
Test6Part2_adj = Test6Part2_adj - [0 mean(Test6Part2(:,2:4))-mean(Test6Part1(:,2:4))]; % Adjust positions
Test6Part2_adj(:,1) = Test6Part2_adj(:,1) + Test6Part1_adj(end,1);

figure(3);
hold on;
grid on;
axis equal
view(50,50);
plot3(Test6Part1(:,2),Test6Part1(:,3),Test6Part1(:,4),'r');
plot3(Test6Part2_adj(:,2),Test6Part2_adj(:,3),Test6Part2_adj(:,4),'b');

figure(4)
hold on
plot(Test6Part1_adj(:,1),Test6Part1_adj(:,2),'b')
plot(Test6Part2_adj(:,1),Test6Part2_adj(:,2),'r')

%% Combine vectors
test6Data = [Test6Part1_adj;Test6Part2_adj];
test6Data_dt = mean(diff(test6Data(:,1)));
test6Data(:,1) = 1:length(test6Data);

%% Normalize position to start position
test6Data(:,2:4) = test6Data(:,2:4) - test6Data(1,2:4);

%% Convert position from cm to mm
%test6Data(:,2:4) = test6Data(:,2:4).*10;

%% Reorient plot
% Current X is desired -Y; current Y is desired X
test6Data(:,2:4) = [test6Data(:,3),-test6Data(:,2),test6Data(:,4)];

%% Plot in order to make sure that we've oriented correctly
%{
figure(5);
hold on;
grid on;
axis equal;
view(50,50);
xlabel('X position, mm');
ylabel('Y position, mm');
zlabel('Z position, mm');
for n = 1:100:size(test6Data,1)
    scatter3(test6Data(n,2),test6Data(n,3),test6Data(n,4),'k');
    drawnow;
end
%}

%% Examine time differences
timeVect = [Test6Part1(:,1); ((Test6Part2(:,1)-Test6Part2(1,1))+Test6Part1(end,1))];
timeVect = timeVect - timeVect(1,:);
timeVect = timeVect/10e6;
timeVect_diff = [0;diff(timeVect)];
dt_mean = mean(timeVect_diff);
dt_stdev = std(timeVect_diff);
% Remove +3sd outliers
timeVect_diffRed = timeVect_diff.*([timeVect_diff <= dt_mean + 3*dt_stdev]);

figure(6);
ax1 = axes('Position',[0 0 1 1],'Visible','off');
ax2 = axes('Position',[.1 .2 .8 .7]);

axes(ax2)
semilogy(ax2,timeVect, timeVect_diff);%,timeVect,timeVect_diffRed);
hold on;
grid on;
semilogy(ax2,timeVect,timeVect_diffRed,':');
title({'ISO 9283 Pose Repeatability:';'Vive Tracker dT'});
xlabel('Sample time (s)');
ylabel('dT (s)');
legend('Unfiltered','<3 S.D. Outliers Removed')

axes(ax1);
descr = {['Mean dT = ',num2str(dt_mean), ' s   |   dT Std. Dev = ',num2str(dt_stdev),' s']};
text(0.32,0.08,descr)

hold off

%% Clear unneeded variables
curvars = {curvars{:},'test6Data'};
clearvars('-except', curvars{:});
clear curvars;