function closeTIFStream()
%% function closeTIFStream()
%   Close TIF File stream
%
%% NOTES
%   Code here is a cut&paste from previous snippet in endAcquisition() -- Vijay Iyer 9/20/09
%
%% CREDITS
%   Created 9/20/09, by Vijay Iyer
%% ***************************************************************************

global state

try
    if ~isempty(state.files.tifStream)
        close(state.files.tifStream);
        state.files.tifStream = [];
        state.files.fileCounter=state.files.fileCounter+1;
        updateGUIByGlobal('state.files.fileCounter');
        updateFullFileName(0);
    end
catch
    delete(state.files.tifStream,'leaveFile');
    errordlg('Failed to close an open TIF stream. A file may be corrupted.');
    state.files.tifStream = [];
end