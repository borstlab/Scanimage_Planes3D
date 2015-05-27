function updateZoom(active)
%UPDATEZOOM Handler for updates to zoom value
%% SYNTAX
%   updateZoom()
%   updateZoom(active)
%       active: Logical value indicating, when true, to put configuration values into effect. Otherwise, changes are for display only. Default (if omitted) is TRUE.
%
%% NOTES
%   Changes to zoom can affect the fill fraction (line period) and servo delay parameters, under the new 'zoom-indexed parameter array' scheme -- Vijay Iyer 1/21/09
%   
%% CHANGES
%   VI092808A: Handle minZoom case handling here -- Vijay Iyer 9/28/08
%   VI121008A: Make sure changes are reflected on the configuration GUI -- Vijay Iyer 12/10/08
%   VI121708A: Handle zoom warning correctly in controls that are inactive -- Vijay Iyer 12/17/08
%   VI122908A: Invoke msPerLine callback when updating its value -- Vijay Iyer 12/29/08
%   VI122908B: Select correct msPerLine/FF values from array -- Vijay Iyer 12/29/08
%   VI123108A: state.internal.baseZoomFactor is now state.acq.baseZoomFactor (again), and its GUI callback should be invoked -- Vijay Iyer 12/31/08
%   VI010209A: Invoke setPockelsAcqParameters to update Pockels acquisition parameters based on current FF/msPerLine values -- Vijay Iyer 1/2/09
%   VI010609A: updateZoomStrings() is now invoked and defined here -- Vijay Iyer 1/6/09
%   VI010909A: Correct names of msPerLine controls -- Vijay Iyer 1/09/09
%   VI012109A: Handle new servoDelayArray and incrementMultiplierArray, while doing away with msPerLineArray and fillFractionArray -- Vijay iyer 1/23/09
%   VI012909A: Update the current scanDelay and acqDelay values; only update scanDelay if sawtooth scanning -- Vijay Iyer 1/28/09
%   VI043009A: Update the frame rate var/display when zoom is updated (potentially updating ms/line) -- Vijay Iyer 4/30/09
%
%% CREDITS
%    Created 9/26/08 by Vijay Iyer -- Janelia Farm Research Campus
%% ****************************************************************************

global state gh

%%%VI012109A
if nargin < 1
    active = true;
end

%%%VI092808A%%%%%%
if state.acq.zoomFactor < state.acq.minZoomFactor 
    state.acq.zoomFactor = state.acq.minZoomFactor;
    updateZoomStrings;
end
%%%%%%%%%%%%%%%%%%

activeZoomWarnControls = [gh.mainControls.zoomfrac gh.mainControls.zoomones gh.mainControls.zoomtens gh.mainControls.zoomhundreds];
%inactiveZoomWarnControls = [gh.configurationGUI.etMsPerLine gh.configurationGUI.pmFillFrac gh.configurationGUI.etScanDelay gh.configurationGUI.etAcqDelay]; %VI121708A, VI010909A

if state.acq.zoomFactor >= state.acq.baseZoomFactor   %all's good   %VI123108A
    
    %%%VI012109A: Removed %%%%%%%%%%
    %     state.acq.fillFraction = state.internal.fillFractionArray(end); %Get highest FF
    %     state.acq.msPerLine = state.internal.msPerLineArray(end); %Get shortest line period
    %     state.internal.numPosSlopePoints = state.internal.posSlopePointsArray(end);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%VI012109A %%%%%%
    state.internal.acqDelayGUI = state.internal.acqDelayArray(end);
    state.internal.fillFractionGUI = state.internal.fillFractionGUIArray(end);     
    if ~state.acq.bidirectionalScan %VI012909A
        state.internal.scanDelayGUI = state.internal.scanDelayArray(end);
    end
    %%%%%%%%%%%%%%%%%%%    

    set(activeZoomWarnControls,'BackgroundColor',[1 1 1]);
    %set(inactiveZoomWarnControls,'BackgroundColor',get(0,'defaultUIControlBackgroundColor')); %VI121708A  
else %need to use lower FF and higher line period, associated with current zoom factor

    %%%VI012109A: Removed %%%%%%%%%%
    %     state.acq.fillFraction = state.internal.fillFractionArray(idx); %VI122908B, VI123108A
    %     state.acq.msPerLine = state.internal.msPerLineArray(idx); %VI122908B, VI123108A
    %     state.internal.numPosSlopePoints = state.internal.posSlopePointsArray(idx); %VI122908B, VI123108A
    %%%%%%%%%%%%%%%%%%%
    
    %%%VI012109A %%%%%%%
    idx = max(1,length(state.internal.fillFractionGUIArray) - (state.acq.baseZoomFactor - round(state.acq.zoomFactor)));
    state.internal.acqDelayGUI = state.internal.acqDelayArray(idx);  
    state.internal.fillFractionGUI = state.internal.fillFractionGUIArray(idx); 
    if ~state.acq.bidirectionalScan %VI012909A
        state.internal.scanDelayGUI = state.internal.scanDelayArray(idx);
    end
    %%%%%%%%%%%%%%%%%%%%
    
    set(activeZoomWarnControls,'BackgroundColor',[1 1 .7]);   
    %set(inactiveZoomWarnControls,'BackgroundColor',get(0,'defaultUIControlBackgroundColor') - [0  0 .2]); %VI121708A
end

[state.acq.fillFraction, state.acq.msPerLine] = decodeFillFractionGUI(state.internal.fillFractionGUI);
updateGUIByGlobal('state.internal.fillFractionGUI'); %VI121008A
updateGUIByGlobal('state.acq.msPerLine'); %VI121008A, VI122908A, VI012809A
updateFrameRate(); %VI043009A

%%%VI012909A%%%%%%%%%%%%%%%%%
state.acq.acqDelay = state.internal.acqDelayGUI * 1e-6; 
updateGUIByGlobal('state.internal.acqDelayGUI');
if state.acq.bidirectionalScan
    updateBidiScanDelay();
else
    state.acq.scanDelay = state.internal.scanDelayGUI * 1e-6;
    updateGUIByGlobal('state.internal.scanDelayGUI');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%VI010209A: Update Pockels Acq parameters, which depend on current/actual FF and line period settings
%setPockelsAcqParameters(); %VI012709A

%VI010609A: Handle the display of zoom level
updateZoomStrings();


function updateZoomStrings
% This functin updates the strings in the zoom buttons
% to be correct with respect to the 
% actual zoom setting.

global state gh
zoomstr= num2str(state.acq.zoomFactor);

%NOTE - This is a kludge based on idea of converting to string and extracting arraywise, which made sense pre fractional-zoom
%       Should probably use strfind() now to search for decimal point in num2str value OR use num2str formatting options
if length(zoomstr)==1
    zoomstr=['00' zoomstr '.0'];
elseif length(zoomstr)==2
    zoomstr=['0' zoomstr '.0'];
elseif length(zoomstr)==3
    if strcmp(zoomstr(2),'.')
        zoomstr=['00' zoomstr];
    else
        zoomstr=[zoomstr '.0'];
    end
elseif length(zoomstr)==4
    zoomstr=['0' zoomstr];
elseif length(zoomstr)>5
    zoomstr=zoomstr(1:5);
end

state.acq.zoomhundreds=str2num(zoomstr(1));
state.acq.zoomtens=str2num(zoomstr(2));
state.acq.zoomones=str2num(zoomstr(3));
state.acq.zoomfrac=str2num(zoomstr(5));
updateGUIByGlobal('state.acq.zoomhundreds');
updateGUIByGlobal('state.acq.zoomtens');
updateGUIByGlobal('state.acq.zoomones');
updateGUIByGlobal('state.acq.zoomfrac');


