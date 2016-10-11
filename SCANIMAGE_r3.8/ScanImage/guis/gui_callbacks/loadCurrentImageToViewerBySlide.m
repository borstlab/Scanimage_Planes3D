function loadCurrentImageToViewerBySlide
global gh state

state.imageViewing.totalFrames = size(state.imageViewing.loadedImage,3);
updateGUIByGlobal('state.imageViewing.totalFrames');
y = size(state.imageViewing.loadedImage,1);
x = size(state.imageViewing.loadedImage,2);	
set(gh.currentImageViewerGUI.axis1, 'Ylim', [1 y], 'XLim', [1 x]);

if state.imageViewing.totalFrames == 1
	set( gh.currentImageViewerGUI.currentFrameSlider, 'Max', 1.00001, 'SliderStep',[1 1]);
else
	set( gh.currentImageViewerGUI.currentFrameSlider, 'Max', state.imageViewing.totalFrames, ...
		'SliderStep', [1/(state.imageViewing.totalFrames-1) 1/(state.imageViewing.totalFrames-1)]);
end

set(state.imageViewing.currentImageBeingViewed, 'CData', state.imageViewing.loadedImage(:,:,state.imageViewing.currentFrame));
		
