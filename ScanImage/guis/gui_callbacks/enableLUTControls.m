function enableLUTControls
global state gh

numberedControls = {'whiteSlideChan' 'blackSlideChan' 'whiteEditChan' 'blackEditChan'};

for i = 1:state.init.maximumNumberOfInputChannels
    if state.acq.(sprintf('imagingChannel%d',i)) || state.acq.(sprintf('maxImage%d',i))
        cellfun(@(x)set(gh.imageControls.(sprintf('%s%d',x,i)),'Enable','on'), numberedControls);
        set(gh.imageControls.(sprintf('text%d',2*i-1)),'Enable','on');
        set(gh.imageControls.(sprintf('text%d',2*i)),'Enable','on');
    else
        cellfun(@(x)set(gh.imageControls.(sprintf('%s%d',x,i)),'Enable','off'), numberedControls);
        set(gh.imageControls.(sprintf('text%d',2*i-1)),'Enable','off');
        set(gh.imageControls.(sprintf('text%d',2*i)),'Enable','off');
    end
end
