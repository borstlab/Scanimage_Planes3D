function  [hAx,hIm,chan] = si_selectImageFigure()
%% function  [hAx,hIm,chan] = si_selectImageFigure()
%Allows user a few seconds to select a valid ScanImage image figure to interact with
%
%% CHANGES
%   VI051310A: Add the hIm output, as some users of this function require this
%
%% CREDITS
%   Created 2/19/09, by Vijay Iyer
%% ************************************************************

global state

hAx = [];
chan = [];
hIm = []; %VI051310A
 
% the zoom/pan tools interfere with selection--turn them off...
zoom(state.hSI.hROIDisplayFig,'off'); 
pan(state.hSI.hROIDisplayFig,'off');

if ~isempty(state.internal.defaultImageTarget)
    if isinf(state.internal.defaultImageTarget) && ~isempty(state.internal.mergeimage)
        hFig = ancestor(state.internal.mergeimage,'figure');
        if strcmpi(get(hFig,'Visible'),'on')
            hAx = ancestor(state.internal.mergeimage,'axes');
            hIm = state.internal.mergeimage; %VI051310A
            return;
        end
    elseif ~isempty(state.internal.imagehandle(state.internal.defaultImageTarget))
        hFig = ancestor(state.internal.imagehandle(state.internal.defaultImageTarget),'figure');
        if strcmpi(get(hFig,'Visible'),'on')
            hAx = ancestor(state.internal.imagehandle(state.internal.defaultImageTarget),'axes');
            chan = state.internal.defaultImageTarget;
            hIm = state.internal.imagehandle(state.internal.defaultImageTarget); %VI051310A
            return;
        end
    end
end                   

selTimer = timer('TimerFcn',@timerFcn,'StartDelay',5);
aborted = false;

%Create dummy figure/axes to divert gcf/gca
hf = figure('Visible','off');
ha = axes('Parent',hf);

%Determine valid figure targets
imageHandles = [state.internal.imagehandle(:); state.internal.mergeimage];
figHandles = [];
for i=1:length(imageHandles)
    figHandles = [figHandles; ancestor(imageHandles(i),'figure')];
end

% add the roiDisplayGUI to the list of valid handles
figHandles = [figHandles; state.hSI.hROIDisplayFig];

%Wait for user to sele
setStatusString('Select Image Figure');
start(selTimer);
while ~aborted      
    currFig = get(0,'CurrentFigure');
    [tf,loc] = ismember(currFig,figHandles);
    if tf        
        hAx = get(currFig,'CurrentAxes');
        if loc <= state.init.maximumNumberOfInputChannels
            chan = loc;
        end
        hIm = findobj(hAx,'Type','image'); %VI051310A
        break;
    end     
    pause(0.5);
end

if aborted
    setStatusString('No Image Selected');
else
    setStatusString('');
end

%Clean up
delete(hf);
stop(selTimer);
delete(selTimer);

    function timerFcn(hObject,eventdata)
        aborted = true;        
    end            

end





% if isempty(findobj(0,'Type','axes')) %no axes exists    
%     logval = false;
% else
%     ax = gca; %may not be on the current figure!
%     
%     imageHandles = [state.internal.imagehandle(:); state.internal.mergeimage];
%     axHandles = [];
%     for i=1:length(imageHandles)
%         axHandles = [axHandles; ancestor(imageHandles(i),'axes')];
%     end
%     
%     logval = ismember(ax,axHandles);
% end
% 
% if ~logval
%     setStatusString('Select Figure First!');
% end
%         




