function playCurrentMovie
global gh state

endframe = size(state.imageViewing.loadedImage,3);
for movieCounter = 1:endframe
	if state.imageViewing.movieFlag == 0
		state.imageViewing.currentFrame = movieCounter;
		updateGUIByGlobal('state.imageViewing.currentFrame');
		set(state.imageViewing.currentImageBeingViewed, 'CData',state.imageViewing.loadedImage(:,:,movieCounter));
		pause(.04);
	else
		state.imageViewing.movieFlag = 0;
		set(gh.currentImageViewerGUI.showMovie, 'String', 'Play Movie');
		break
	end
end
set(gh.currentImageViewerGUI.showMovie, 'String', 'Play Movie');