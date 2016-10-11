function reloadLastImage
global gh state

state.imageViewing.loadedImage = state.imageViewiing.oldLoadedImage;
state.imageViewing.currentFrame = 1;
updateGUIByGlobal('state.imageViewing.currentFrame');
loadCurrentImageToViewerBySlide;