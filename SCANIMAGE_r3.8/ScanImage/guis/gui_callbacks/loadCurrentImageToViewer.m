function loadCurrentImageToViewer
global gh state

for channelCounter = 1:state.init.maximumNumberOfInputChannels
	channelOn = eval(['state.imageViewing.channel' num2str(channelCounter)]);
	if channelOn == 1	
		%state.imageViewing.loadedImage = state.acq.acquiredData{channelCounter};
		y = size(state.acq.acquiredData{1}{channelCounter},1);
		x = size(state.acq.acquiredData{1}{channelCounter},2);	
		set(gh.currentImageViewerGUI.axis1, 'Ylim', [1 y], 'XLim', [1 x]);
		state.imageViewing.totalFrames = length(state.acq.acquiredData);
		updateGUIByGlobal('state.imageViewing.totalFrames');
		if state.imageViewing.totalFrames == 1
			set( gh.currentImageViewerGUI.currentFrameSlider, 'Max', 1.00001, 'SliderStep',[1 1]);
		else
			set( gh.currentImageViewerGUI.currentFrameSlider, 'Max', state.imageViewing.totalFrames, ...
				'SliderStep', [1/(state.imageViewing.totalFrames-1) 1/(state.imageViewing.totalFrames-1)]);
		end
		
		set(state.imageViewing.currentImageBeingViewed, 'CData', state.acq.acquiredData{state.imageViewing.currentFrame}{channelCounter});	
	end
end
