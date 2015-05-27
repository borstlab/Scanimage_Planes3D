function updateClim
%% function updateClim
% Function that executes whenever the lookup table is changed.
% It sets the axis propert CLim to the new Values.
%
%% NOTES
%   This function also creates/maintains additional 'state' variable fields (e.g. state.internal.lowPixelValue) which vectorize the low/high limits
%   This makes access of the limits more convenient for many functions. 
%% CHANGES
%   VI030809A: Ensure that high pixel value exceeds low pixel value -- Vijay Iyer 3/8/09
%   
%% CREDITS
% Written By: Thomas Pologruto
% Cold Spring Harbor Labs
% January 9, 2002
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
global state gh

state.internal.highPixelValue=[];
state.internal.lowPixelValue=[];
for i = 1:state.init.maximumNumberOfInputChannels
    %%%VI030809A%%%%%%%%%%%%%%%%%%%
    highPixelValue = state.internal.(['highPixelValue' num2str(i)]);
    lowPixelValue = state.internal.(['lowPixelValue' num2str(i)]);
    
    if highPixelValue <= lowPixelValue
        highPixelValue = lowPixelValue + 1;
        state.internal.(['highPixelValue' num2str(i)]) = highPixelValue;
        updateGUIByGlobal(['state.internal.highPixelValue' num2str(i)]);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
	state.internal.highPixelValue = [state.internal.highPixelValue highPixelValue]; %VI030809A
	state.internal.lowPixelValue = [state.internal.lowPixelValue lowPixelValue]; %VI030809A
    
end

for i = 1:state.init.maximumNumberOfInputChannels
	if isfield(state.acq, 'imagingChannel')
		if state.acq.imagingChannel(i)
			try
				set([state.internal.axis(i) state.internal.maxaxis(i)], 'CLim', [state.internal.lowPixelValue(i) state.internal.highPixelValue(i)]);
			end 
		end
	end
end

state.hSI.roiUpdateViewCLim();

