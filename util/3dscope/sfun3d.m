function [sys, x0, str, ts] = sfun3d(t,x,u,flag,ax,varargin)

%SFUN3D S-function that acts as an X-Y-Z scope using MATLAB plotting functions.
%   This M-file is designed to be used in a Simulink S-function block.
%   It draws a line from the previous input point and the current point.
%
%   NOTE: this is a new version of sfunxyz. It has more natural inputs
%   that is (x1,y1,z1, x2,y2,z2 ... instead of x1,x2, y1,y2, z1,z2 ...)
%   and has the LineStyle and Marker properties as additional parameters,
%   so for versions 2016a and later users should try to use this one 
%   instead of the older sfunxyz.
%
%   See also sfunxy, sfunxys.

%   Copyright 2017 The MathWorks, Inc.
%   Based on original work by Andy Grace (May-91), Wes Wang (Apr-93, Aug-93, 
%   Dec-93), Craig Santos (Oct-96), and Giampiero Campa (Apr-94, Nov-15).

switch flag
    
    case 0
        % note this also sets up the local figure
        [sys,x0,str,ts] = mdlInitializeSizes(ax,varargin{:});
        
        % callbacks due to actions by the user on the block
        callbacks={
            'CopyFcn',       'if exist(''sfun3d'',''file''), sfun3d([],[],[],''CopyBlock''); end';
            'DeleteFcn',     'if exist(''sfun3d'',''file''), sfun3d([],[],[],''DeleteBlock''); end';
            'LoadFcn',       'if exist(''sfun3d'',''file''), sfun3d([],[],[],''LoadBlock''); end';
            'NameChangeFcn', 'if exist(''sfun3d'',''file''), sfun3d([],[],[],''NameChange''); end';
            };
        
        % set callbacks as block properties (for these flag is a string)
        for i=1:length(callbacks)
            if ~strcmp(get_param(gcbh,callbacks{i,1}),callbacks{i,2})
                set_param(gcbh,callbacks{i,1},callbacks{i,2})
            end
        end
        
    case 2
        sys = mdlUpdate(t,x,u,flag);
        
    % The following four callbacks are set by "case 0" above
    case 'NameChange'
        % get the figure associated with this block, if it's valid, change
        % the name of the figure
        FigHandle=get_param(gcbh,'UserData');
        if ishandle(FigHandle)
            set(FigHandle,'Name',get_param(gcbh,'Name'));
        end
        
    case {'CopyBlock', 'LoadBlock'}
        % Initialize the block's UserData such that a figure is not associated with the block
        set_param(gcbh,'UserData',-1);
        
    case 'DeleteBlock'
        % Get the figure handle associated with the block, if it exists, delete
        % the figure.
        FigHandle=get_param(gcbh,'UserData');
        if ishandle(FigHandle)
            delete(FigHandle);
            set_param(gcbh,'UserData',-1);
        end
        
    case {3,9}
        sys=[];
        
    otherwise
        if ischar(flag)
            errmsg=sprintf('Unhandled flag: ''%s''', flag);
        else
            errmsg=sprintf('Unhandled flag: %d', flag);
        end
        
        error(errmsg);
        
end
% end sfunxy

%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function
% Here is used also to initialize the figure and its objects.
%=============================================================================
function [sys,x0,str,ts] = mdlInitializeSizes(ax,varargin)

vrs=version;
if str2double(vrs(1:3))<9.0
    error('This S-Function (sfun3d.m) works only within MATLAB versions 2016a and later. For older versions, please delete any existing version of 3Dscope (), (re)install it on this MATLAB version, and use the legacy S-Function ''sfunxyz.m''.');
end

if length(ax)~=6, error('Axes limits must be defined.'); end

sizes=simsizes; % initializes everything to zero
sizes.NumInputs = 3*fix(varargin{2})+1;
sizes.NumSampleTimes = 1;

% return values (note that in earlier versions there was no sampling time)
sys=simsizes(sizes);x0=[];str=[];ts=[varargin{1} 0];

% do the figure initialization:
FigHandle=get_param(gcbh,'UserData');
if isempty(FigHandle) || ~ishandle(FigHandle)
    % the figure doesn't exist, create one
    FigHandle = figure(...
        'Units',          'pixel',...
        'Position',       [100 100 400 300],...
        'Name',           get_param(gcbh,'Name'),...
        'Tag',            'SIMULINK_3DGRAPH_FIGURE',...
        'NumberTitle',    'off',...
        'IntegerHandle',  'off',...
        'Toolbar',        'none',...
        'Menubar',        'none');
else
    % otherwise clear it
    clf(FigHandle);
end

% create the objects:

% get varargin arguments
nmax=fix(varargin{2});
CameraPosition=varargin{3};
if varargin{4}, GdSwitch='On'; else, GdSwitch='Off'; end

% store the block's handle in the figure's UserData
ud.Block=gcbh;

% axes
ud.XYZAxes = axes(FigHandle);
cord=get(ud.XYZAxes,'ColorOrder');
set(ud.XYZAxes,'Visible','on','Xlim', ax(1:2),'Ylim', ax(3:4),'Zlim', ax(5:6),'CameraPosition',CameraPosition,'XGrid',GdSwitch,'YGrid',GdSwitch,'ZGrid',GdSwitch);

% line
ud.XYZLine = [];
for n=1:nmax
    ud.XYZLine = [ud.XYZLine animatedline(ud.XYZAxes,'LineStyle',varargin{5},'Marker',varargin{6},'Color',cord(1+mod(n-1,size(cord,1)),:))];
end

% labels
xlabel('X Axis');ylabel('Y Axis');zlabel('Z Axis');

% title
ud.XYZTitle  = get(ud.XYZAxes,'Title');
set(ud.XYZTitle,'String','X Y Z Plot');

% Associate the figure with the block, and set the figure's UserData.
set_param(gcbh,'UserData',FigHandle); % store figure handle in block's UD
set(FigHandle,'UserData',ud); % store ud in figure's userdata
% end mdlInitializeSizes

%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step stuff.
% Here is used only to add another point to the lines.
%=============================================================================
function sys=mdlUpdate(~,~,u,~)

% always return empty, as there are no states
sys = [];

% Locate the figure window associated with this block.  If it's not a valid
% handle (it may have been closed by the user), then return.
FigHandle=get_param(gcbh,'UserData');
if isempty(FigHandle) || ~ishandle(FigHandle), return, end

% get userdata structure
ud = get(FigHandle,'UserData');

% add points to each line, or clear
nmax=length(ud.XYZLine);

if u(length(u)) == 1;
    for i=1:nmax
        clearpoints(ud.XYZLine(i));
    end
else
    for i=1:nmax
        addpoints(ud.XYZLine(i),u(3*(i-1)+1),u(3*(i-1)+2),u(3*(i-1)+3));
    end
end



% end mdlUpdate
