function loadXZProjection
global gh state

if size(state.imageViewing.loadedImage,3) > 1
	state.imageViewiing.oldLoadedImage = state.imageViewing.loadedImage;
end

state.imageViewing.loadedImage = collapse(state.imageViewing.loadedImage, 'XZ');
state.imageViewing.currentFrame = 1;
updateGUIByGlobal('state.imageViewing.currentFrame');
loadCurrentImageToViewerBySlide;