function setAxisGrids(axisHandle, bins)
% places grid lines on axis handle
set(axisHandle, 'XTickLabelMode', 'manual', 'YTickLabelMode', 'manual');
axisHandleCopy=axisHandle;
setImagesToWhole;
for axCounter=1:length(axisHandle)
    axisHandle=axisHandleCopy(axCounter);
    xlims = get(axisHandle, 'XLim');
    ylims = get(axisHandle, 'YLim');
    
    totalX = xlims(2)-xlims(1);
    totalY = ylims(2)-ylims(1);
    
    xbins = round((totalX)/bins);
    ybins=round((totalY)/bins);
    nbins=bins+1;
    
    xticks = linspace(xlims(1),xlims(2),nbins);
    yticks = linspace(ylims(1),ylims(2),nbins);
    
    set(axisHandle, 'XLim', xlims, 'XTick', xticks);
    set(axisHandle, 'YLim', ylims, 'YTick', yticks);
    set(axisHandle, 'XGrid', 'on', 'YGrid', 'on', 'XColor', 'b', 'YColor', 'b', 'GridLineStyle', '-','Layer','Top');
end
