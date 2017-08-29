% 2152FinalProjectProcessing
%{
Julian Leland
2.152, Spring 2017

This script contains scratch code used in JL's 2.152 final project.
%}

%% Change directory to correct directory and load files
cd experimental/j1testbench/2152FinalProj/
load('MotorFreqResp4.mat');

%% Convert frequencies in cleaned data to radians
% NOTE - DON'T NEED TO DO THIS! Already in rad/s. GRRR.
%dsaData_comp(:,1) = dsaData_comp(:,1)*2*pi; 

%% Wrap experimentally measured phase angle
% NOTE - Doesn't really help. The phase angle that's measured after it
% flips is mostly gibberish.
curphase = 0;
dsaData_new = dsaData_comp;
for n = 1:size(dsaData_new,1)
    if dsaData_new(n,3) > curphase + 90
        dsaData_new(n,3) = dsaData_new(n,3)-360
    end
    if dsaData_new(n,3) <= curphase
        curphase = dsaData_new(n,3);
    end
end

%% Superimpose motor frequency response on estimated system Bode plot
% Requires that the model of the brushed DC motor (tf1) and the cleaned dsa data, be in the workspace
w = logspace(-1,4,500);
[mag_est, ph_est] = bode(tf1,w);
mag_flat = reshape(mag_est,1,length(mag_est));
ph_flat = reshape(ph_est,1,length(ph_est));

figure(1);
subplot(2,1,1);
loglog(dsaData_comp(:,1),dsaData_comp(:,2))
hold on;
loglog(w, mag_flat);
title('Magnitude');
xlabel('Frequency, rad/s');
ylabel('Magnitude, abs');
legend('Measured','Estimated TF');
hold off

subplot(2,1,2);
semilogx(dsaData_comp(:,1),dsaData_new(:,3))
hold on;
loglog(w, ph_flat);
title('Phase');
xlabel('Frequency, rad/s');
ylabel('Phase, deg');

titletxt = {'DC Motor Frequency Response';'Measured vs. Estimated TF'}
suptitle(titletxt);
hold off;

%% Perform calculations with motor measured parameters to estimate J
R = 15.5; % R = 15.5 ohms
V = 11.9; % Vsupply = 11.9 V for tests
I = 0.23; % Isupply = 0.23 A for tests

wMax = 432.667; % Max angular velocity = 432.667 rad/s

tRise = 0.02434; % Measured rise time constant, sec.

Kb = (V - I*R)/wMax

B = (Kb*I)/wMax

J_motor = (tRise*(R*B + Kb^2))/R

%% Estimate reflected inertia of backlash hub components
J_la = 1.07e-4; % kg-m^2 - inertia around rotating axis of inner hub
J_lb = 3.59e-3; % kg-m^2 - inertia around rotating axis of outer hub

k_gbox = 1/20; % Reduction in gearbox
k_belt = (25/120) * (60/120); % Reduction in belt drive

k_ref = ((k_gbox*k_belt)^2)
J_leff = (J_la + J_lb)* k_ref

%% Extract X, Y, Z data from error plots
% Use this code to extract data from plots
% Make sure the figure you want is open first!
plotnum = 2; % Pick which line you'd like to extract

h = gcf;

axesObjs = get(h,'Children');
dataObjs = get(axesObjs,'Children');
lineObjs = findobj(dataObjs,'type','line');

xdata = get(lineObjs(plotnum),'XData');
ydata = get(lineObjs(plotnum),'YData');

% Plot to check
figure;
plot(ydata)

%% Calculate root-sum-square error for given region of plot
startval = 17090;
ydata(isnan(ydata)) = 0; % Set NaN values to zero
y_rmsErr = rms(ydata(startval:end)')