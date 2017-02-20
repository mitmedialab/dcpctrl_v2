%{
iso9283testsegmenter.m
Julian Leland, MIT Media Lab
2017-02-12

Originally written as PSet 2 for 12.444 Signal Processing Module, Fall 2016

This code leverages basic signal processing techniques to assist in
analyzing a dataset generated using ISO 9283 robot performance tests. This
code is submitted to fulfill the requirements of the 12.444 Module 2
(Signal Processing) project.


GOALS: The goals of this project are as follows:

1) Write parsing code that reads through each dataset and detects/logs 1) 
times where all three signals (X, Y and Z) become stationary, and 2) times 
where all three signals are stationary at the defined "starting point" for 
the dataset. Time: 5-8 hrs
 - This code should be able to parse either dataset listed in the code.
 - I may optionally input the number of expected waypoints per cycle if 
   needed to detect waypoints accurately.
 - I will (try to) use only filters that we've discussed in class. I'm 
   particularly interested in using higher-order Gaussian filters to detect 
   starting/stopping, to avoid calculating derivatives for the entire 
   dataset.
 - I already have code that runs the ISO 9283 pose repeatability test on my 
   (parsed) data, so the output format will be set to match that code.

2) Return basic statistics on measured data. My first goal is to return 
FFTs of the data measured during motion and during stopped phases, to get a 
sense of the frequency content of the noise inherent in my data. I expect 
to see some low-frequency (1-10 Hz) vibration from the system engine, but 
will be curious what else I can find.

3) Write code to create a randomly-generated dataset with selectable 
parameters (number of stopping points, amount of noise), which I can then 
use to experiment with robustness of processing code.
NOTES: 

%}

%% Set up convenience functions & variables
%datarange = 7590:7800; % Range of data to plot for focus plots. Useful for debugging missed edges
datarange = 1:500;
colors = colormap(gcf); % Set up colormap
close all;
gendata = 0; % Flag to generate dataset
fixdata = 0; % Flag to apply manual fixes to data.
dt = 1/3; % Sample time for system, sec.

%% Load data
% Shorter dataset
% load('ExampleISO9283Data.mat')
% Longer dataset, proper ISO 9283
data_temp = load('ExampleISO9283Data_Longer.mat','ExampleISO9283Data_Longer')
data = data_temp.ExampleISO9283Data_Longer;
data_bkp = data; % Create backup copy, just in case.
clear data_temp;

%% OPTIONAL: Generate dataset
if gendata
    
    % Set up waypoints. These should be X/Y/Z pairs
    xwaypts = [100 400 250 600];
    ywaypts = [450 -250 450 -250];
    zwaypts = [1200 -60 1200 -500];
    waypts = [xwaypts' ywaypts' zwaypts'];
    
    % Set up path parameters
    twaypt = 30; % Time in seconds to move between waypoints
    tdwell = 5; % Time in seconds to dwell at each endpoint
    reps = 2; % Number of repeats of set
    sigfreq = 0.5; % Frequency of sine signal to add, Hz
    sigamp = [0 5 2]; % Amplitude of sine signal in [X Y Z]
    noiseSNR = 100; % SNR for white noise
    
    % Interpolate between waypoints to create seed set
    data_gen = []; % Empty vector for generated data
    t_prev = 0;
    for n = 1:length(waypts)
        % Generate dwell points
        t = [0:dt:tdwell] + t_prev;
        xsteps = ones(1,length(t))*waypts(n,1);
        ysteps = ones(1,length(t))*waypts(n,2);
        zsteps = ones(1,length(t))*waypts(n,3);
        data_gen = [data_gen;[xsteps' ysteps' zsteps' t']];
        t_prev = max(t);
        
        % Generate move points
        t = [0:dt:twaypt] + t_prev;
        if n == length(waypts) % If we're at end of waypoint set, wrap around
            xsteps = interp1([min(t), max(t)],[waypts(n,1) waypts(1,1)],t);
            ysteps = interp1([min(t), max(t)],[waypts(n,2) waypts(1,2)],t);
            zsteps = interp1([min(t), max(t)],[waypts(n,3) waypts(1,3)],t);
        else
            xsteps = interp1([min(t), max(t)],[waypts(n,1) waypts(n+1,1)],t);
            ysteps = interp1([min(t), max(t)],[waypts(n,2) waypts(n+1,2)],t);
            zsteps = interp1([min(t), max(t)],[waypts(n,3) waypts(n+1,3)],t);
        end
        data_gen = [data_gen;[xsteps' ysteps' zsteps' t']];
        t_prev = max(t);
    end
    
    % Repeat set desired number of times
    seed_datagen = data_gen;
    tmax = max(data_gen(:,4));
    for n = 1:reps
        seed_datagen(:,4) = seed_datagen(:,4) + tmax;
        data_gen = [data_gen;seed_datagen];
    end
    
    % Inject sine signal into signal
    data_genfft = fft(data_gen(:,1:3));
    sine_fft = fft(sin(sigfreq*2*pi*data_gen(:,4)));
    data_gen(:,1:3) = [ifft(data_genfft(:,1)+sigamp(1)*sine_fft),ifft(data_genfft(:,2)+sigamp(2)*sine_fft),ifft(data_genfft(:,3)+sigamp(3)*sine_fft)];
    
    % Inject Gaussian white noise into signal
    data_gen(:,1:3) = awgn(data_gen(:,1:3),noiseSNR,'measured');
    
    % Add index vector and set dataset name appropriately
    data = [(0:length(data_gen)-1)',data_gen(:,1),data_gen(:,2),data_gen(:,3)];
end

%% Sort dataset by index vector
% Turns out that the data isn't properly indexed by default. Who knew??
data = sortrows(data,1);

%% Add time vector to data
data(:,5) = data(:,1).*dt;

%% Display data in plot
figure(1);
subplot(1,2,1)
plot3(data(:,2),data(:,3),data(:,4));
xlabel('X position, mm');
ylabel('Y position, mm');
zlabel('Z position, mm');
grid on;
axis equal
subplot(1,2,2);
plot(data(:,5),data(:,2),data(:,5),data(:,3),data(:,5),data(:,4));
xlabel('Time, s');
ylabel('Position, mm');
legend('X','Y','Z');
suptitle('Laser Tracker Data: 3D & Per-Axis'); 

%% Take FFT of data to see what frequencies exist
% Because sampling frequency was so low - 3 Hz - we are unlikely to be able
% to extract any meaningful information about vehicle vibration, etc.
% However, we may be able to see some oscillations of the boom.

% Normalize each row around data mean.
data_norm = [(data(:,2)-mean(data(:,2))),(data(:,3)-mean(data(:,3))),(data(:,4)-mean(data(:,4))),data(:,5)];
plot(data_norm(:,4),data_norm(:,1),data_norm(:,4),data_norm(:,2),data_norm(:,4),data_norm(:,3));
xlabel('Time, s');
ylabel('Position, mm');
legend('X','Y','Z');

% IMPORTANT: For fftshift, need to specify which dimension to shift along.
data_fft = fftshift(fft(data_norm(:,1:3)),1); % FFT only cols 1-3
L = length(data_fft)/2;

% Generate wave number plot
% figure(2);
% plot(-L:(L-1),abs(data_fft(:,2)));
% xlabel('Wave Number');
% ylabel('Amplitude');

data_ffthalf = abs(data_fft((L+1):end,:)/L); % Get positive half of spectrum 
% data_ffthalf is normalized by dividing by number of samples(2L), and then
% multiplied by 2 to get full power --> we just divide by L
freq_ffthalf = [0:(1/(2*L)):1/2-1/(2*L)]*(1/dt); % Frequencies for spectrum

% Plot full overlapped spectrum
figure(3);
plot(freq_ffthalf,data_ffthalf);
%xlim([-0.01,0.4]); % Reduce X-axis limits to focus on low frequencies
ylim([0,max(max(data_ffthalf(:,2)),max(data_ffthalf(:,3)))]);
grid on;
xlabel('Frequency, Hz');
ylabel('Magnitude, normalized');
title('Positive Frequency Spectrum');
legend('X','Y','Z');

% Plot each axis individually
figure(4);
subplot(3,1,1);
plot(freq_ffthalf,data_ffthalf(:,1));
xlabel('Frequency, Hz');
ylabel('X Direction - Amplitude');
xlim([-0.01,0.4]); % Reduce X-axis limits to focus on low frequencies
%xlim([1,1.5]);
grid on;
subplot(3,1,2);
plot(freq_ffthalf,data_ffthalf(:,2));
xlabel('Frequency, Hz');
ylabel('Y Direction - Amplitude');
xlim([-0.01,0.4]); % Reduce X-axis limits to focus on low frequencies
%xlim([1,1.5]);
grid on;
subplot(3,1,3);
plot(freq_ffthalf,data_ffthalf(:,3));
xlim([-0.01,0.5]); % Reduce X-axis limits to focus on low frequencies
%xlim([1,1.5]);
xlabel('Frequency, Hz');
ylabel('Z Direction - Amplitude');
grid on;
suptitle('Positive Frequency Spectrum - X/Y/Z Directions');

%% Take FFT of data - MATLAB-recommended method, just a sanity check
%{
Fs = 1/dt;
X = data_norm(:,2); % Look at Y motion 
L = length(X);
Y = fft(X);
P2 = abs(Y/L)';
P1 = P2(:,1:L/2+1); % Grab half the spectrum
P1(:,2:end-1) = 2*P1(:,2:end-1); % Double everything after DC to get full amplitude
f = Fs*(0:(L/2))/L;
figure(12);
plot(f,P1);
xlim([-0.01,0.4]);
%}

%% Filter signals to detect edges - low-pass
% Reference for this section: Lecture Notes Class 4, Sai Ravela, Appendix 2
% Create Butterworth low-pass filter to reject all HF noise
wp = 0.2; % Set passband edge to 0.2 Hz
ws = 0.9; % Set stopband edge to 0.9 Hz
[n,wn] = buttord(wp,ws,3,60); % Calculate Butterworth filter cutoff and order
[b,a] = butter(n,wn); % Calculate filter coefficients

% Look at zero-normalized signals - we can later extrapolate results to
% non-normalized signals
data_normfilt = [filtfilt(b,a,data_norm(:,1:3)),data_norm(:,4)]; % Include time

% Plot one axis to verify performance
figure(5)
%plot(data_normfilt(:,4),data_normfilt(:,2),data_norm(:,4),data_norm(:,2))
plot(data_normfilt(datarange,4),data_normfilt(datarange,2),data_norm(datarange,4),data_norm(datarange,2))
xlabel('Time, s');
ylabel('Position, mm');
title('Z Position: Low-Pass Filtering');
legtitle = sprintf('wp = %0.2f',wp);
legend(legtitle,'Unfiltered');

%% Filter signals to detect edges using Gaussian 2nd-order filter
% Set Gaussian filter parameters
len = 50;
x = -len:(len-1);
sig = 0.8;

% Create filter
g = fspecial('gaussian',[1 length(x)],sig);
dg = sig.*gradient(g);
edge = conv(dg,data_normfilt(:,2),'full');

% Convolve filter with data to detect edges
d_datanormfilt = [conv(dg,data_normfilt(:,1)),conv(dg,data_normfilt(:,2)),conv(dg,data_normfilt(:,3))];
d_datanormfilt = [d_datanormfilt(len:end-len,:),data_normfilt(:,4)];

% Plot all three axes against derivatives to verify performance
figure(6)
plotrange = [datarange]; % Optional setting for plotrange
subplot(3,1,1);
plotyy(data_normfilt(datarange,4),data_normfilt(datarange,1),d_datanormfilt(plotrange,4),d_datanormfilt(datarange,1));
xlabel('Time, s');
ylabel('X Position, mm');
legend('Position','Derivative');
grid on;
subplot(3,1,2);
plotyy(data_normfilt(datarange,4),data_normfilt(datarange,2),d_datanormfilt(plotrange,4),d_datanormfilt(datarange,2));
xlabel('Time, s');
ylabel('Y Position, mm');
legend('Position','Derivative');
grid on;
subplot(3,1,3);
plotyy(data_normfilt(datarange,4),data_normfilt(datarange,3),d_datanormfilt(plotrange,4),d_datanormfilt(datarange,3));
xlabel('Time, s');
ylabel('Z Position, mm');
legend('Position','Derivative');
grid on;
suptitle('Position & Derivative - X/Y/Z Directions');

%% Detect "stopped" state for each axis
% Compare derivative to some threshold to determine what counts as
% "stopped"
data_stopped = []; % Empty vector for stopped state
stop_dthresh = [2 2 2]; % Thresholds for determining "stopped" state for X, Y and Z derivative data
data_stopped = [(abs(d_datanormfilt(:,1)) <= stop_dthresh(1)),(abs(d_datanormfilt(:,2)) <= stop_dthresh(2)),(abs(d_datanormfilt(:,3)) <= stop_dthresh(3)),d_datanormfilt(:,4)];

% Filter 1: Smooth stopped state data to reject spikes
%{
len = 12; % Shorter filter this time
x = -len:(len-1);
sig = 1000;
g = fspecial('gaussian',[1 length(x)],sig);
data_stopped = [conv(g,+data_stopped(:,1)),conv(g,+data_stopped(:,2)),conv(g,+data_stopped(:,3))];
data_stoppedfilt = [data_stopped(len:end-len,:),d_datanormfilt(:,4)];
%}

% Filter 2 - DIY logical filter
% This filter operates on logical data instead of continuous-valued data.
% It effectively averages over a set window to remove small spikes from
% data, while still ensuring that data transitions occur in a binary
% fashion (either stopped or moving).
%%{
filtlen = 10; % Number of samples on either side to average. Total filter length is 2*filtlen+1.
data_stoppedfilt = []; % Empty vector for filtered data
for n = filtlen+1:length(data_stopped)-filtlen
    cursum(n,:) = sum(data_stopped(n-filtlen:n+filtlen,1:3),1);
    data_stoppedfilt(n,:) = (cursum(n,:) > floor((2*filtlen + 1)/2));
end
data_stoppedfilt(end:length(data_stopped),:) = 0; % Pad end with zeroes
%data_stoppedfilt = logical(data_stoppedfilt);
data_stoppedfilt(:,4) = d_datanormfilt(:,4); % Add time vector
%}

% Plot stopped index against position derivative
figure(7);
subplot(3,1,1);
suptitle('Logical Low-Pass Filter');
[AX, H1, H2] = plotyy(data_normfilt(datarange,4),(data_normfilt(datarange,1)),data_stoppedfilt(datarange,4),data_stoppedfilt(datarange,1));
xlabel('Time,s')
ylabel(AX(1),'d\_Position - X Axis, mm/s');
ylabel(AX(2),'Stopped Flag, n/a');
hold(AX(2),'on');
plot(AX(2),data_stopped(datarange,4),data_stopped(datarange,1),'--');
ylim(AX(2),[-0.1, 1.1]);
legend('Data','Filtered Stop','Unfiltered Stop');
hold off
subplot(3,1,2);
[AX, H1, H2] = plotyy(data_normfilt(datarange,4),(data_normfilt(datarange,2)),data_stoppedfilt(datarange,4),data_stoppedfilt(datarange,2));
xlabel('Time,s')
ylabel(AX(1),'d\_Position - Y Axis, mm/s');
ylabel(AX(2),'Stopped Flag, n/a');
hold(AX(2),'on');
plot(AX(2),data_stopped(datarange,4),data_stopped(datarange,2),'--');
ylim(AX(2),[-0.1, 1.1]);
hold off
subplot(3,1,3);
[AX, H1, H2] = plotyy(data_normfilt(datarange,4),(data_normfilt(datarange,3)),data_stoppedfilt(datarange,4),data_stoppedfilt(datarange,3));
xlabel('Time,s')
ylabel(AX(1),'d\_Position - Z Axis, mm/s');
ylabel(AX(2),'Stopped Flag, n/a');
hold(AX(2),'on');
plot(AX(2),data_stopped(datarange,4),data_stopped(datarange,3),'--');
ylim(AX(2),[-0.1, 1.1]);
hold off

%% Detect stopped state for all axes & create edge flag showing when this occurs
stopdata = data_stoppedfilt(:,1) & data_stoppedfilt(:,2) & data_stoppedfilt(:,3);
stopdata = +stopdata;
stopdata(:,2) = [0;diff(stopdata)];
stopdata(:,3) = data_stoppedfilt(:,4);

figure(8)
hold on;
[AX, H1, H2] = plotyy(data(datarange,5),data(datarange,2:4),stopdata(datarange,3),stopdata(datarange,1));
hold(AX(2),'on');
plot(AX(2),stopdata(datarange,3),stopdata(datarange,2),'k');
ylim(AX(2),[-0.1, 1.1]);
xlabel('Time, s');
ylabel('Position, mm');
legend('X','Y','Z','Stopped','Transition');
title('System Stopped Detection');

hold off;

%% OPTIONAL: Add/remove additional start and stop states to data to capture missed steps
if fixdata
    % NOTE: All start/stop values here are *index* values. If you want to put
    % in time values, divide the value by dt.
    addstart = [1729/dt]; % Add additional start pos states in form [pos1start;pos2start;...]
    addstop = [1735/dt]; % Add additional stop pos states in form [pos1stop;pos2stop;...]

    % Add new positions
    stopdata(addstart,2) = 1;
    stopdata(addstop,2) = -1;

    % Remove pairs of start and stop positions to join segments. Remember,
    % segments should be ordered stop, followed by start!
    remstop = [7651]; % Remove stop positions in form [pos1stop;pos2stop;...]
    remstart = [7663]; % Remove start positions in form [pos1start;pos2start;...]

    % Remove positions
    stopdata(remstop,2) = 0;
    stopdata(remstart,2) = 0;
    
    % Plot area of interest with modified start and stop positions
    figure(8)
    clf;
    hold on;
    [AX, H1, H2] = plotyy(data(datarange,5),data(datarange,2:4),stopdata(datarange,3),stopdata(datarange,1));
    hold(AX(2),'on');
    plot(AX(2),stopdata(datarange,3),stopdata(datarange,2),'k');
    ylim(AX(2),[-0.1, 1.1]);
    xlabel('Time, s');
    ylabel('Position, mm');
    legend('X','Y','Z','Stopped','Transition');
    title('System Stopped Detection - Manual Edits');
    hold off;
end

%% Detect how many different stopped states there are
% Set up thresholds and find start and stop locations
statethresh = 55; % Threshold in mm per axis to detect difference between two states
startpos = find(stopdata(:,2) == 1); % Find locations where we are *starting* a new stopped state
stoppos = find(stopdata(:,2) == -1); % Find locations where we are *ending* a new stopped position

% Iterate through stopped regions and calculate averages until we find a
% repeat
breakflag = 0;
avgpos = [];
for n = 1:length(startpos)
    curpos = mean(data(startpos(n):stoppos(n),2:4));
    if n >= 2
        for q = 1:size(avgpos,1)
            if abs(avgpos(q,:)-curpos(1,:)) <= statethresh*ones(1,3)
                fprintf('Repeated state found after %d states! Terminating search.\n',n-1);
                numstates = n-1;
                breakflag = 1;
                break;
            else
            end
        end
    end
    avgpos(n,:) = curpos;
    if breakflag
        break;
    end
end

% For rest of dataset, create new data_out vector which includes location
% number
data_out = data(:,1:5); % Grab index, position and timestamp from existing data vector
data_out(:,6:7) = 0; % Last 2 columns are zeroes
for n = 1:length(startpos)
    curindx = mod(n-1,numstates)+1;
    data_out(startpos(n):stoppos(n),6) = curindx;
end

% Finally, add move index vector
moveindx = 0; % Start index for which move we're on
data_out(1,7) = 0;
for n = 2:length(data_out)
    data_out(n,7) = moveindx;
    if ((data_out(n,6) == 1) && (data_out(n-1,6) == 0))
        moveindx = moveindx + 1;
    end
end

%% Final plots of movement with shaded sections indicating different poses
% Full plot to detect errors
figure(9)
% Plot run position data
hold on;
plot(data_out(:,5),data(:,2:4))
xlabel('Time, s');
ylabel('Position, mm');
title('X/Y/Z Position - Waypoints Highlighted')

% Get X and Y limits
xl = get(gca,'xlim');
yl = get(gca,'ylim');

% Create patches in a different color for each pose
offst = 10; % Number of indices to offset label in X dir
for n = 1:length(startpos)
    patch([data_out(startpos(n),5) data_out(stoppos(n),5) data_out(stoppos(n),5) data_out(startpos(n),5)],[yl(1) yl(1) yl(2) yl(2)],colors(data_out(startpos(n),6)+3)); 
end
set(gca,'children',flipud(get(gca,'children')))
hold off;

% Detail plot to diagnose errors
figure(10)
hold on;
%plot(data_out(datarange,5),data(datarange,2:4))
plot(data_out(datarange,1),data(datarange,2:4))
xlabel('Time, s');
ylabel('Position, mm');
legend('X','Y','Z');

% Figure out what closest stopped zone is that is on plot
idxlim = get(gca,'xlim'); % Get index limits
startpos_idx = [min(find(startpos >= idxlim(1))),max(find(startpos <= idxlim(2)))];

% Create stop zones and numbers
for n = startpos_idx(1):startpos_idx(2)
    text(data_out(startpos(n)+offst,1), mean(xl), num2str(n));
    patch([data_out(startpos(n),1) data_out(stoppos(n),1) data_out(stoppos(n),1) data_out(startpos(n),1)],[yl(1) yl(1) yl(2) yl(2)],colors(data_out(startpos(n),6)+3));
end
set(gca,'children',flipud(get(gca,'children')))
plot(data_out(datarange,1),ones(length(data_out(datarange,1)))*mean(xl)); % Create horizontal line to measure position in time
    
