function dataMissedFcn(~,~)
%DATAMISSEDFCN Callback function invoked upon an Analog Input data missed event
%
%% NOTES
%   Beginning in r3.7, this handler is used if AI Task is stopped in middle for any error -- the most likely being an overflow error
%
%   TODO: Add DAQmx error message decoding to ensure it really is an overflow error before presenting error dialog. This will require using the status parameter in the Done Event callback.
%
%% CREDITS
%   Created 10/24/09, by Vijay Iyer
%% ************************************************

global gh

abortCurrent(false); %Do not set status string
setStatusString('Data Rate Too High!');

%Show dialog
hDlg = errordlg({'Input data was not processed fast enough - acquisition aborted!'; ...
    ''; ...
    'To avoid this error in the future, do one or more of the following: ' ; ...
    '  1) Reduce the data rate (frames per second, number channels, etc)'; ...
    '  2) Reduce the amount of background applications/activity'; ...
    '  3) Reduce the amount of GUI interaction during acquisition'} ...
    , 'Data Rate Too High','modal');

%Set position to just below current configurationGUI position (whether visible or not)
dlgPosn = getpixelposition(hDlg);
mainPosn = getpixelposition(gh.mainControls.figure1);
dlgPosn(1) = mainPosn(1);
dlgPosn(2) = mainPosn(2)-dlgPosn(4)-25;
setpixelposition(hDlg,dlgPosn);

%Bind deleteFcn callback
set(hDlg,'DeleteFcn',@closeErrDlg);

    function closeErrDlg(hObject,eventdata)
        setStatusString('');        
    end

end




