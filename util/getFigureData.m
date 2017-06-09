%{
getFigureData.m
Julian Leland, MIT Media Lab, 2017-05-22

This function extracts the data from a MATLAB plot. WIP.
%}

% Get current figure handle (assumes plot is open)
h = gcf;

% Search handle for all possible figure types
lineObjs = findobj(h,'type','line');
stairObjs = findobj(h,'type','stair');
barObjs = findobj(h,'type','bar');
areaObjs = findobj(h,'type','area');
stemObjs = findobj(h,'type','stem');

% Figure out what type of plot we have
type = 0;
if sum(size(lineObjs)) > 2
    type = 1;
    curObjs = lineObjs;   
elseif sum(size(stairObjs)) > 2
    type = 2;
    curObjs = stairObjs;
elseif sum(size(barObjs)) > 2
    type = 3;
    curObjs = barObjs;
elseif sum(size(areaObjs)) > 2
    type = 4;
    curObjs = areaObjs;
elseif sum(size(stemObjs)) > 2
    type = 5;
    curObjs = stemObjs;
end

% Extract data accordingly
xdata = [];
ydata = [];
for n = 1:length(curObjs)
   xdata(:,n) = curObjs(n).XData;
   ydata(:,n) = curObjs(n).YData;
end

% Note: In some plots (esp. from Simulink) there are NaNs floating around
% that need to be eliminated. Use the following code as an example for how
% to find and remove these.

%{
absEncoderData(find(isnan(absEncoderData(:,1))),:)=[]
relEncoderData(find(isnan(relEncoderData(:,1))),:)=[]
%}