function loadImageFromFileToViewer
global gh state

[fname, pname] = uigetfile('*.tif', 'Choose image to load');
if fname < 1
	return
else
	state.imageViewiing.oldLoadedImage=0;
	state.imageViewing.loadedImage = opentif([pname fname]);
	state.imageViewing.currentFrame = 1;
	updateGUIByGlobal('state.imageViewing.currentFrame');
	y = size(state.imageViewing.loadedImage,1);
	x = size(state.imageViewing.loadedImage,2);	
	set(gh.currentImageViewerGUI.axis1, 'Ylim', [1 y], 'XLim', [1 x]);
	state.imageViewing.totalFrames = size(state.imageViewing.loadedImage,3);
	updateGUIByGlobal('state.imageViewing.totalFrames');
	set(state.imageViewing.currentImageBeingViewed, 'CData', state.imageViewing.loadedImage(:,:,state.imageViewing.currentFrame));
	if state.imageViewing.totalFrames == 1
			set( gh.currentImageViewerGUI.currentFrameSlider, 'Max', 1.00001, 'SliderStep',[1 1]);
		else
			set( gh.currentImageViewerGUI.currentFrameSlider, 'Max', state.imageViewing.totalFrames, ...
				'SliderStep', [1/(state.imageViewing.totalFrames-1) 1/(state.imageViewing.totalFrames-1)]);
	end
end
