function [ qraw2PWMMap ] = qraw2pwmmapgen_at40gw()
%QRAW2PWMMAPGEN Generate maps between raw joint velocities (sensor
%velocities) and PWM duty cycles for lookup tables.

%{
qraw2pwmmapgen_at40gw.m
Julian Leland, MIT Media Lab, 2017-02-08

This function creates 

INPUT: None

OUTPUT:
  
%}

%% Set up lookup tables for each array
% The following data has been gathered from a range of tests conducted to
% examine the relationship between PWM and joint sensor velocity. For raw
% test data and further information on these tests, please see: 
% /MDCP2/Design/Testing/TestData_RAW/PWM-VelocityCharacterization

% If future testing is conducted & additional data needs to be added, do it
% in the next few rows! Include the name of the original dataset, and
% format as [PWMS; qraws]

%% J1 Data

% t12_data: Extracted from J1 testing conducted 2016-05-26
t12_data = [97500,102500,107500,112500,117500,122500,127500,132500,137500,142500,147500,152500,155500;2810.63591863809,4297.14839013777,5283.01861970186,6408.10606106621,8603.99688472601,10604.9453179151,13029.6872137488,14347.8727623619,18184.3140858011,19266.1440775901,19257.0113096179,19252.1146739202,19256.9076357704];
t12_data(1,:) = t12_data(1,:)/160000; % Normalize to 0-1 scale

% J1_data_t: Extracted from J1 testing conducted 2016-06-24
J1_data_t = [89500,90500,91500,92500,93500,94500,95500,96500,97500,97500,98500,99500,100500,101500,102500,102500,103500,104500,105500,106500,107500,107500,112500,117500,122500,127500,132500,137500,142500,147500,152500,155500;509.600092789176,742.272014976205,940.988769671326,1230.31458101117,1460.81090033077,1764.14527522036,2042.84431350311,2376.07222243526,2605.55130117320,2677.08894303910,2978.32124735305,3266.72173566556,3471.47341579186,3836.77779760222,3954.82876970721,4095.75261609013,4366.89524542669,4643.26975049721,4805.93586635405,5017.10275686239,5120.18122852591,5205.01531166883,6259.05651369581,7926.80468568284,9861.74994202617,12113.0032038265,15406.8780329398,17197.9481319556,17224.8539102370,17204.8233507887,17176.5627837154,17159.1907103349];
J1_data_t(1,:) = J1_data_t(1,:)/160000; % Normalize to 0-1 scale

% Combine & sort datasets
J1_qraw2PWM_comb = sortrows([t12_data,J1_data_t]');

% Average duplicate datapoints
PWMS = unique(J1_qraw2PWM_comb(:,1));
J1_qraw2PWM_avg = zeros(length(PWMS),2);
J1_qraw2PWM_avg(:,1) = PWMS;
for n = 1:length(PWMS)
    mask = [J1_qraw2PWM_comb(:,1) == J1_qraw2PWM_avg(n,1)];
    J1_qraw2PWM_avg(n,2) = sum(mask.*J1_qraw2PWM_comb(:,2))/sum(mask);
end

% Clamp to lowest elbow value
J1_qraw2PWM_1s = J1_qraw2PWM_avg; % Output is a single-sided value
maxVel = min(max(t12_data(2,:)),max(J1_data_t(2,:))); % Get the smallest of the maximum raw velocities
mask = J1_qraw2PWM_avg(:,2) >= maxVel;
J1_qraw2PWM_1s(mask,2) = maxVel;

% We do not have data for reversal of J1 (and we have no reason to think
% that it won't work the same in both directions). Now need to flip
% mapping to cover 0-50% region
J1_qraw2PWM_2s = zeros(size(J1_qraw2PWM_1s));
J1_qraw2PWM_2s(:,1) = flipud(1 - J1_qraw2PWM_1s(:,1));
J1_qraw2PWM_2s(:,2) = flipud(-J1_qraw2PWM_1s(:,2));

% Create final output mapping
J1_qraw2PWM = [J1_qraw2PWM_2s;[0.5,0];J1_qraw2PWM_1s];

%% J2 Data
% No data has been gathered yet for J2! Put in dummy values here
J2_qraw2PWM = [0, -1;
               0.5, 0;
               1, 1];

%% J3 Data

% J3_data_t: From characterization 2016-05-26
J3_data_t = [5000,5000,9500,9500,14500,14500,19500,19500,24500,24500,29500,29500,30000,34500,34500,34500,39500,39500,39500,44500,44500,44500,49500,49500,49500,54500,54500,54500,59500,59500,59500,64500,64500,64500;1.38580490542509,1.39406949035492,1.39127867907708,1.39146933498644,1.38931683459174,1.39629260643984,1.39101781541242,1.39336968014771,1.38876362903483,1.39355042925167,1.38206987229613,1.39153350864535,1.33506228331504,1.19722218937955,1.21082925245997,1.23283378424461,0.959592222589914,0.965335316916831,1.00151285962958,0.747353192364262,0.759019358445773,0.782256516925729,0.576002357354778,0.576889910292259,0.583287008686338,0.452697575377901,0.455021728578302,0.461124022235874,0.352368498032283,0.354626022635881,0.359595526626594,0.248853168758356,0.249285383542165,0.252710433982257];
J3_data_t(1,:) = J3_data_t(1,:)/160000; % Normalize to 0-1 scale

% Combine & sort datasets
J3_qraw2PWM_comb = sortrows([J3_data_t]');

% Average duplicate datapoints
PWMS = unique(J3_qraw2PWM_comb(:,1));
J3_qraw2PWM_avg = zeros(length(PWMS),2);
J3_qraw2PWM_avg(:,1) = PWMS;
for n = 1:length(PWMS)
    mask = [J3_qraw2PWM_comb(:,1) == J3_qraw2PWM_avg(n,1)];
    J3_qraw2PWM_avg(n,2) = sum(mask.*J3_qraw2PWM_comb(:,2))/sum(mask);
end

% Clamp to lowest elbow value
J3_qraw2PWM_1s = J3_qraw2PWM_avg; % Output is a single-sided value
maxVel = min(max(J3_data_t(2,:))); % Get the smallest of the maximum raw velocities
mask = J3_qraw2PWM_avg(:,2) >= maxVel;
J3_qraw2PWM_1s(mask,2) = maxVel;

% We do not have data for reversal of J3 - flip mapping to cover 50-100%
% region.
J3_qraw2PWM_2s = zeros(size(J3_qraw2PWM_1s));
J3_qraw2PWM_2s(:,1) = flipud(1 - J3_qraw2PWM_1s(:,1));
J3_qraw2PWM_2s(:,2) = flipud(-J3_qraw2PWM_1s(:,2));

% Create final output mapping
J3_qraw2PWM = sortrows([J3_qraw2PWM_2s;[0.5,0];J3_qraw2PWM_1s]);

%% J4 Data

% J4for_data_t & J4rev_data_t: From characterization 2016-05-26
J4for_data_t = [35000,35000,38000,38000,43000,43000,48000,48000,53000,53000,58000,58000,63000,63000;0.976777187212586,0.994276948834789,0.975840046636136,0.987177718635892,0.839074527450802,0.842608342482293,0.657015793533094,0.659515536302646,0.539514774120757,0.541517791414585,0.401029548240214,0.407740311603782,0.257694841363199,0.258358550035785];
J4for_data_t(1,:) = J4for_data_t(1,:)/160000; % Normalize to 0-1 scale
J4rev_data_t = [97500,97500,102500,102500,107500,107500,112500,112500,117500,117500,122500,122500,127500,127500,132500,132500,137500,137500,140000,140000;-0.202879042231395,-0.175969537425480,-0.292486222486330,-0.241470592969498,-0.369707982491424,-0.284164677572071,-0.444577573206986,-0.341999210373294,-0.563653496214737,-0.424110791028247,-0.661752456920627,-0.482714560022262,-0.765035156482492,-0.567224145470589,-0.887490501766593,-0.670773079760858,-0.934863357547488,-0.674890988934688,-0.932724568464702,-0.669866895710671];
J4rev_data_t(1,:) = J4rev_data_t(1,:)/160000; % Normalize to 0-1 scale

% dataset: From characterization 2016-06-27
dataset = [0.912771533716100,0.917651610717602,0.876211325810059,0.695205001263779,0.526430003879369,0.375625634875123,0.401532383198876,0.363312541662857,0.336755906059947,0.307498524928652,0.273115288527678,0.236826020869078,0.250317205468011,0.213033682595767,0.186152932557130,0.154762810180918,0.130531299544044,0.108688450664471,0.0838926242656773,0.0746996540332431,0.0533177439777823;35000,38000,43000,48000,53000,58000,58000,59000,60000,61000,62000,63000,63000,64000,65000,66000,67000,68000,69000,70000,71000];
dataset = [dataset(2,:);dataset(1,:)]; % Reverse order of rows
dataset(1,:) = dataset(1,:)/160000; % Normalize to 0-1 scale

% Combine & sort datasets
% Note that we add the 50%/zero velocity entry at this point
J4_qraw2PWM_comb = sortrows([J4for_data_t,J4rev_data_t,dataset,[0.5;0]]');

% Average duplicate datapoints
PWMS = unique(J4_qraw2PWM_comb(:,1));
J4_qraw2PWM_avg = zeros(length(PWMS),2);
J4_qraw2PWM_avg(:,1) = PWMS;
for n = 1:length(PWMS)
    mask = [J4_qraw2PWM_comb(:,1) == J4_qraw2PWM_avg(n,1)];
    J4_qraw2PWM_avg(n,2) = sum(mask.*J4_qraw2PWM_comb(:,2))/sum(mask);
end

% Clamp max to lowest elbow value
J4_qraw2PWM_1s = J4_qraw2PWM_avg;
maxVel = min(max(J4for_data_t(2,:)),max(dataset(2,:))); % Get the smallest of the maximum raw velocities
mask = J4_qraw2PWM_avg(:,2) >= maxVel;
J4_qraw2PWM_1s(mask,2) = maxVel;

% Clamp min to lowest elbow value
minVel = max(min(J4rev_data_t(2,:))); % Get the smallest of the maximum raw velocities
mask = J4_qraw2PWM_avg(:,2) <= minVel;
J4_qraw2PWM_1s(mask,2) = minVel;

% Create final output mapping
J4_qraw2PWM = J4_qraw2PWM_1s;

%% Combine mappings
qraw2PWMMap = {J1_qraw2PWM,J2_qraw2PWM,J3_qraw2PWM,J4_qraw2PWM};