function loadCurrentImageToNewWindow
global state gh
y = size(state.imageViewing.loadedImage,1);
x = size(state.imageViewing.loadedImage,2);
h=figure('DoubleBuffer', 'on','Position',[129 222 x y]);
colormap(scim_colorMap('gray',8)); %VI112210A
clim = get(gh.currentImageViewerGUI.axis1,'CLim');
imagesc(state.imageViewing.loadedImage(:,:,state.imageViewing.currentFrame));
set(gca, 'CLim', clim, 'YTickLabelMode', 'manual', 'XTickLabelMode', 'manual','XTickLabel', [], 'YTickLabel', [], 'DataAspectRatioMode', 'manual', 'Position', [0 0 1 1]);