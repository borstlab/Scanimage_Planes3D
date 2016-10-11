function makeImageFigures
%% function makeImageFigures
%
%  Function that creates the various acquisition/display windows used by ScanImage
%%
%  Written by: Thomas Pologruto
%  Cold Spring Harbor Labs
%  January 16, 2001
%
%% MODIFICATIONS
%    Added global declarations in callbacks. - T. O'Connor 12/23/03
%   VI022108A Vijay Iyer 2/21/08 - Added color merge figure
%   VI022108B Vijay Iyer 2/21/08 - Explicitly set the OpenGL renderer bug-mode, turning on/off a workaround for anomalies with non-normal EraseMode. 
%                                   With some boards, at least, the workaround causes more problems than the bug.
%   VI022308A Vijay Iyer 2/23/08 - Switched away from OpenGL to default 'painters' renderer. Found to handl resizing better, without speed degradation.
%   VI120108A Vijay Iyer 12/01/08 - Bind KeyPressFcn callback via a function handle rather than by name. genericKeyPressFcn() now uses eventdata argument, which only works when a function handle is used.
%   VI030309A Vijay Iyer 03/03/09 - Pass along figure from which 'Select ROI' was chosen from the dropdown menu 
%   VI091009A Vijay Iyer 09/10/09 - Initialize EraseMode to 'none' for all images (except ROI display). This provides best performance. Only reason to use 'normal' would be if using OpenGL and/or if using image crosshairs.
%   VI102209A Vijay Iyer 10/22/09 -- Use state.internal.storedLinesPerFrame where appropriate
%   VI102609A Vijay Iyer 10/26/09 -- Use state.internal.scanAmplitudeX/Y in lieu of state.acq.scanAmplitudeX/Y, as the internal value is now used to represent the actual command voltage
%   VI103009A: Don't display 'Making image windows' to status string -- Vijay Iyer 10/30/09
%   VI103009A: Don't display 'Making image windows' to status string -- Vijay Iyer 10/30/09
%   VI092310A: Eliminate assigmnent of ButtonDownFcn to now-deprecated figureButtonOverCallback(); the datatip tool supersedes this now -- Vijay Iyer 9/23/10
%   VI112310A: Initialize Colormap property of figures with gray(256); this is just a temporary setting, as it gets overridden downstream by value of figureColorMap# state var -- Vijay Iyer 11/23/10
%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2002
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%

global state gh

%status=state.internal.statusString; %VI103009A
%setStatusString('Making image windows...'); %VI103009A

startImageData = zeros(state.internal.storedLinesPerFrame, state.acq.pixelsPerLine); %VI102209A
startImageData = uint8(startImageData);
axisPosition = [0 0 1 1];

% This loop sets up the aspect ratios for the figures
if state.internal.scanAmplitudeSlow ~= 0 & state.internal.scanAmplitudeFast ~= 0 %VI102609A
    aspectRatioF = abs(state.internal.imageAspectRatioBias*state.internal.scanAmplitudeSlow/state.internal.scanAmplitudeFast);  %VI102609A
    aspectRatio = (state.acq.pixelsPerLine/state.internal.storedLinesPerFrame)*aspectRatioF; %VI102209A
else % Line scan so make the image accordingly....
    aspectRatioF=-1;
    aspectRatio=-1;
end

%Set the figure positions....
figurePosition=ones(state.init.maximumNumberOfInputChannels,4); %initialize the array...
maxfigurePosition=ones(state.init.maximumNumberOfInputChannels,4); %initialize the array...
for i = 1:state.init.maximumNumberOfInputChannels
    if aspectRatioF <= 1 & aspectRatioF > 0
        eval(['figurePosition(i,:) = [state.internal.figurePositionX' num2str(i) ' state.internal.figurePositionY' ...
                num2str(i) ' state.internal.figureWidth' num2str(i) ' aspectRatioF*state.internal.figureHeight' num2str(i) '];']);
        eval(['maxfigurePosition(i,:) = [state.internal.maxfigurePositionX' num2str(i) ' state.internal.maxfigurePositionY' ...
                num2str(i) ' state.internal.maxfigureWidth' num2str(i) ' aspectRatioF*state.internal.maxfigureHeight' num2str(i) '];']);
    elseif aspectRatioF > 1                
        eval(['figurePosition(i,:) = [state.internal.figurePositionX' num2str(i) ' state.internal.figurePositionY' ...
                num2str(i) ' state.internal.figureWidth' num2str(i) '/aspectRatioF state.internal.figureHeight' num2str(i) '];']);
        eval(['maxfigurePosition(i,:)  = [state.internal.maxfigurePositionX' num2str(i) ' state.internal.maxfigurePositionY' ...
                num2str(i) ' state.internal.maxfigureWidth' num2str(i) '/aspectRatioF state.internal.maxfigureHeight' num2str(i) '];']);
    else    %lINESCAN.....
        figurePosition(i,:)=get(state.internal.GraphFigure(i),'position'); %Not clear how this could work (VI022108A)
        maxfigurePosition(i,:)=get(state.internal.MaxFigure(i),'position');%Not clear how this could work (VI022108A)
    end
end

%Handle merge window similarly (VI022108A)
if aspectRatioF <= 1 && aspectRatioF > 0
    mergePosition = [state.internal.mergefigurePositionX state.internal.mergefigurePositionY ...
        state.internal.mergefigureWidth aspectRatioF*state.internal.mergefigureHeight];
  elseif aspectRatioF > 1
      mergePosition = [state.internal.mergefigurePositionX state.internal.mergefigurePositionY ...
          state.internal.mergefigureWidth/aspectRatioF state.internal.mergefigureHeight];
  else    %lINESCAN.....
      mergePosition = get(state.internal.MergeFigure,'position'); %Not clear how this could work
end

% This loop creates the appropriate images, figures and axes.
for i = 1:state.init.maximumNumberOfInputChannels

    state.internal.GraphFigure(i) = figure('Position', figurePosition(i,:) ,'doublebuffer', 'on', 'KeyPressFcn', @genericKeyPressFunction, ... %VI120108A
        'Tag',  ['GraphFigure' num2str(i)], 'Name',  ['Acquisition of Channel ' num2str(i)], 'NumberTitle', 'off',  'MenuBar', 'none', ...
        'CloseRequestFcn', 'set(gcf, ''visible'', ''off'')', 'visible', 'off','ColorMap', gray(256),'ResizeFcn','setImagesToWhole'); %VI112310A
    %Setup UI Context Menus...Right click on image or axis...
    cmenu = uicontextmenu('Parent', state.internal.GraphFigure(i));
    
    %Added the global declarations into the callbacks. Matlab 6.5 seems to
    %require this (or something does anyway, on Volker's rig).
    %uimenu(cmenu, 'Label', 'Select ROI', 'Callback', 'global gh; mainControls(''ROI_Callback'',gh.mainControls.ROI)'); %VI030309A
    uimenu(cmenu, 'Label', 'Select ROI', 'Callback', ['global gh; mainControls(''ROI_Callback'',' num2str(state.internal.GraphFigure(i)) ')']); %VI030309A
    uimenu(cmenu, 'Label', 'Undo', 'Callback', 'global gh; mainControls(''undo_Callback'',gh.mainControls.undo)');
    uimenu(cmenu, 'Label', 'Select LineScan', 'Callback', 'global gh; mainControls(''selectLineScanAngle_Callback'',gh.mainControls.selectLineScanAngle)');
    uimenu(cmenu, 'Label', 'Center', 'Callback', 'global gh; mainControls(''centerOnSelection_Callback'',gh.mainControls.centerOnSelection)');
    uimenu(cmenu, 'Label', 'Goto Reset', 'Callback', 'global gh; mainControls(''pbBase_Callback'',gh.mainControls.pbBase)');
    uimenu(cmenu, 'Label', 'Set Reset', 'Callback', 'global gh; mainControls(''pbSetBase_Callback'',gh.mainControls.pbSetBase)');
    uimenu(cmenu, 'Label', 'Add ROI', 'Callback', 'global gh; mainControls(''addROI_Callback'',gh.mainControls.addROI)');
    uimenu(cmenu, 'Label', 'Toggle LS', 'Callback','global state gh; state.acq.linescan=1-state.acq.linescan;,updateGUIByGlobal(''state.acq.linescan'');,mainControls(''linescan_Callback'',gh.mainControls.linescan);');
  
    state.internal.axis(i) = axes('YDir', 'Reverse','NextPlot', 'add', 'XLim', [-0.5 .5] + [1 state.acq.pixelsPerLine],'YLim', [-0.5 .5] + [1 state.internal.storedLinesPerFrame], ... %VI102209A
        'CLim', [0 1], 'Parent', state.internal.GraphFigure(i), ...
        'YTickLabelMode', 'manual', 'XTickLabelMode', 'manual', 'Position', axisPosition,  ...
        'XTickLabel', [], 'YTickLabel', [], 'DataAspectRatioMode', 'manual',  ...
        'UIContextMenu',cmenu); %VI092310A
    if aspectRatio > 0
        set(state.internal.axis(i),'DataAspectRatio', [aspectRatio 1 1])
    end
    state.internal.imagehandle(i) = image('CData', startImageData, 'CDataMapping', 'Scaled', 'Parent', state.internal.axis(i), ...
        'UIContextMenu',cmenu,'EraseMode','none'); %VI092310A %VI091009A
    
    state.internal.focusimage{i}=[];
end

% Same for the max projection windows
for i = 1:state.init.maximumNumberOfInputChannels
    state.internal.MaxFigure(i) = figure('Position', maxfigurePosition(i,:) ,'doublebuffer', 'on', 'KeyPressFcn', @genericKeyPressFunction, ... %VI120108A
        'Tag',  ['MaxFigure' num2str(i)], 'Name',  ['Max Projection of Channel ' num2str(i)], 'NumberTitle', 'off',  'MenuBar', 'none', ...
        'CloseRequestFcn', 'set(gcf, ''visible'', ''off'')','visible', 'off','ColorMap', gray(256)); %VI112310A
    
    state.internal.maxaxis(i) = axes('YDir', 'Reverse','NextPlot', 'add', 'XLim', [-0.5 .5] + [1 state.acq.pixelsPerLine],'YLim', [-0.5 .5] + [1 state.internal.storedLinesPerFrame],... %VI102209A
        'CLim', [0 1], 'Parent', state.internal.MaxFigure(i), ...
        'YTickLabelMode', 'manual', 'XTickLabelMode', 'manual', 'Position', axisPosition,  ...
        'XTickLabel', [], 'YTickLabel', [], 'DataAspectRatioMode', 'manual'); 
    if aspectRatio > 0
        set(state.internal.maxaxis(i),'DataAspectRatio', [aspectRatio 1 1])
    end
    state.internal.maximagehandle(i) = image('CData', startImageData, 'CDataMapping', 'Scaled', 'Parent', state.internal.maxaxis(i), ...
        'EraseMode','none'); %VI092310A %VI091009A
end
%Make ROI Manager Image
%TPMOD 6/17/03
%roipos=[state.internal.roifigureNewPositionX state.internal.roifigureNewPositionY state.internal.roifigureNewWidth state.internal.roifigureNewHeight];
% state.internal.roifigure = figure('doublebuffer', 'on', 'Position',roipos,...
%         'Tag',  'ROIFigure', 'Name',  'ROI Display Figure', 'NumberTitle', 'off', 'KeyPressFcn', @genericKeyPressFunction,... %VI120108A
%         'CloseRequestFcn', 'set(gcf, ''visible'', ''off'')','visible', 'off','ColorMap', gray(256),... %VI112310A
%         'Visible',state.internal.roifigureVisible);
% state.internal.roiaxis = axes('YDir', 'Reverse','NextPlot', 'add', 'XLim', [-.5 .5],'YLim', [-.5 .5],...
%         'CLim', [0 1], 'Parent',  state.internal.roifigure, 'Color',[0 0 0],...
%         'YTickLabelMode', 'manual', 'XTickLabelMode', 'manual', 'Position', [0 0 1 1],  ...
%         'XTickLabel', [], 'YTickLabel', [], 'DataAspectRatioMode', 'manual');
% state.internal.roiimage = image('CData', startImageData, 'CDataMapping', 'Scaled', 'Parent', state.internal.roiaxis,'XData',[-.5 .5],'YData',[-.5 .5]);
% updateCurrentROI;
%updateMainControlSize;

%Bind to 'New' ROI figure
%state.internal.roifigure = state.hSI.hROIDisplayFig;
% set(state.internal.roifigureNew,'Position',roipos);

%Make Merge window Figure (VI022108A)
state.internal.MergeFigure = figure('Position',mergePosition,'Name','Channel Merge','Renderer','painters','CloseRequestFcn','set(gcf,''Visible'',''off'');',...
    'DoubleBuffer','on','NumberTitle','off','Tag','MergeFigure','MenuBar','none','Visible','off','ResizeFcn','setImagesToWhole');
state.internal.mergeaxis = axes('Parent',state.internal.MergeFigure,'YDir','reverse','XLim', [-0.5 .5] + [1 state.acq.pixelsPerLine],'YLim',[-0.5 .5] + [1 state.internal.storedLinesPerFrame],... %VI102209A
    'DataAspectRatioMode','manual','Position',[0 0 1 1],'XTickLabel',[],'YTickLabel',[]);
if aspectRatio > 0
    set(state.internal.mergeaxis,'DataAspectRatio', [aspectRatio 1 1])
end
state.internal.mergeimage =  image('CData',uint8(zeros(state.acq.pixelsPerLine,state.internal.storedLinesPerFrame,3)),'Parent',state.internal.mergeaxis,'EraseMode','none'); %VI102209A %VI091009A 
%opengl('OpenGLEraseModeBug',state.init.openGLEraseModeBug); %VI022108B, VI022308A

%Wrap-up
%setStatusString(status); %VI103009A %restores status string 
