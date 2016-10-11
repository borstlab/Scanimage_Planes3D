function resetImageProperties(initialize,~)
global state gh
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Function that creates or reformats the images to comply with current mode of operation.
%
% Written By: Thomas Pologruto
% Cold Spring Harbor Labs
% April 3, 2003
%
% Changes:
%   TPMOD_1: Modified 12/31/03 Tom Pologruto - Corrects max figure positions as well
%       as remembering the correct locations of acq. figures
%   TPMOD_2: Modified 12/31/03 Tom Pologruto - Since we cannot compute the proper ratio of X:Y
%       in a linescan mode, we need to tell peopel their figures will not
%       go where they are supposed to...
%   VI022108A: Modified 2/21/08 Vijay Iyer - Added merge figure to list of images reset
%   VI042208A: Modified 4/22/08 Vijay Iyer - Don't beep at the user when unable to reset the Image properties; switch from warning to red fprintf() 
%   VI021109A: Don't recreate teh merge figure while resetting it -- Vijay Iyer 2/11/09
%   VI050509A: Handle linescan based on linescan state variable now -- Vijay Iyer 5/5/09
%   VI050509B: Handle linescan case asepct ratio -- Vijay Iyer 5/5/09
%   VI102209A: Use state.internal.storedLinesPerFrame where appropriate -- Vijay Iyer 10/22/09
%   VI103009A: Don't display 'Making image windows' to status string -- Vijay Iyer 10/30/09
%   VI103009A: Don't display 'Making image windows' to status string -- Vijay Iyer 10/30/09
%   VI110210A: Rename scanAmplitudeX/Y to scanAngularRangeX/Y -- Vijay Iyer 11/2/10
%   VI112911A: Update image crosshair, or lack thereof, on all calls to resetImageProperties() -- Vijay Iyer 11/29/11
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



if nargin < 1 || ishandle(initialize) || isempty(initialize)
	initialize = false;
end

if ~initialize && ~state.hSI.mdlInitialized
    return;
end

% Define the figure and its properties.  USe uint8 since they are al zeros and to save memory.
%status=state.internal.statusString; %VI103009A
%setStatusString('Making image windows...'); %VI103009A
axisPosition = [0 0 1 1];
% This loop sets up the aspect ratios for the figures
%if state.acq.scanAmplitudeY ~= 0 & state.acq.scanAmplitudeX ~= 0 %VI050509A

if state.acq.scanAngleMultiplierFast == 0 || state.acq.scanAngleMultiplierSlow == 0
    aspectRatio = state.acq.pixelsPerLine/state.internal.storedLinesPerFrame;
else
    aspectRatioF = abs(state.internal.imageAspectRatioBias*(state.init.scanAngularRangeReferenceSlow * state.acq.scanAngleMultiplierSlow)/(state.init.scanAngularRangeReferenceFast * state.acq.scanAngleMultiplierFast)); %VI110210A %Aspect ratio can remain based on state.acq.scanAngularRangeFast/Slow, rather than state.internal.scanAmplitudeFast/Slow -- Vijay Iyer 10/26/09
    aspectRatio = (state.acq.pixelsPerLine/state.internal.storedLinesPerFrame)*aspectRatioF; %VI102209A
end

%%%VI050509A: Removed
% else % Line scan so make the image accordingly....    
%     aspectRatioF=-1;
%     aspectRatio=-1;
%     % start TPMOD_2 12/31/03
%     % beep,warning('resetImageProperties: cannot change figure positions in a linescan mode.');
%     fprintf(2,'resetImageProperties: cannot change figure positions in a linescan mode.\n'); %VI052208A %VI050509A: Removed
%     % end TPMOD_2 12/31/03
% end
%%%%%%%%%%%%%%%%%%%

if initialize
    figurePosition=ones(state.init.maximumNumberOfInputChannels,4); %initialize the array...
    maxfigurePosition=ones(state.init.maximumNumberOfInputChannels,4); %initialize the array...
    for i = 1:state.init.maximumNumberOfInputChannels
        if state.acq.imagingChannel(i) || state.acq.maxImage(i)
		
			figs = {'figure' 'maxfigure'};
			for figureName = figs
				figureName = figureName{1};
				x = [figureName 'PositionX' num2str(i)];
				y = [figureName 'PositionY' num2str(i)];
				w = [figureName 'Width' num2str(i)];
				h = [figureName 'Height' num2str(i)];
				eval([figureName 'PositionCurrent = [state.internal.(''' x ''') state.internal.(''' y ''') state.internal.(''' w ''') state.internal.(''' h ''')];']);
                
                if strcmp(figureName,'figure')
                    winName = 'Graph';
                else
                    winName = 'Max';
                end
                
                eval(['set(state.internal.' winName 'Figure(i),''Position'', ' figureName 'PositionCurrent(:));']);
			end
        end
    end
    
    currentMergeFigurePosition = [state.internal.mergefigurePositionX state.internal.mergefigurePositionY state.internal.mergefigureWidth state.internal.mergefigureHeight];
    set(state.internal.MergeFigure,'Position',currentMergeFigurePosition);
end

%Set the figure properties....
for i = 1:state.init.maximumNumberOfInputChannels % Count through all the channels
    if state.acq.imagingChannel(i)	% is thsi one to be imaged?
        set(state.internal.axis(i),'XLim',  [-0.5 .5] + [1 state.acq.pixelsPerLine], 'YLim', [-0.5 .5] +  [1 state.internal.storedLinesPerFrame], 'CLim', [state.internal.lowPixelValue(i) ... %VI102209A
                state.internal.highPixelValue(i)], 'Position', axisPosition);
        set(state.internal.GraphFigure(i),'Visible', 'on');
        set(state.internal.axis(i),'DataAspectRatio', [aspectRatio 1 1]);
    else
        set(state.internal.GraphFigure(i), 'Visible', 'off');
    end
    if state.acq.maxImage(i)	% is thsi one to be imaged?
        set(state.internal.maxaxis(i),'XLim',  [-0.5 .5] + [1 state.acq.pixelsPerLine], 'YLim', [-0.5 .5] + [1 state.internal.storedLinesPerFrame], 'CLim', [state.internal.lowPixelValue(i) ... %VI102209A
                state.internal.highPixelValue(i)], 'Position', axisPosition);
        set(state.internal.MaxFigure(i),'Visible', 'on');
        set(state.internal.maxaxis(i),'DataAspectRatio', [aspectRatio 1 1]);
    else
        set(state.internal.MaxFigure(i), 'Visible', 'off');
    end
end

%Handle merge window similarly (VI022108A)
if state.acq.channelMerge
    set(state.internal.mergeaxis,'XLim', [-0.5 .5] + [1 state.acq.pixelsPerLine],'YLim',[-0.5 .5] + [1 state.internal.storedLinesPerFrame],'DataAspectRatioMode','manual','Position',axisPosition); %VI102209A
    set(state.internal.MergeFigure,'Visible','on');
    set(state.internal.mergeaxis,'DataAspectRatio', [aspectRatio 1 1])
    
%     state.internal.mergeimage =  image('CData',uint8(zeros(state.acq.pixelsPerLine,state.acq.linesPerFrame,3)),'Parent',state.internal.mergeaxis); %VI021109A
else
    set(state.internal.MergeFigure,'Visible','off');
end

updateClim;
%updateMainControlSize;
updateImageBox(); %VI112911A
%setStatusString(status); %VI103009A
