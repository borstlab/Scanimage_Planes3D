function ok = validateAcquisitionDelay()
%% function validateAcquisitionDelay()
%   Verifies that acquisition delay for upcoming acquisition is not too high; if not, it informs user by changing control color and, optionally, a warning dialog
%
%% SYNTAX
%   ok = validateAcquisitionDelay()
%       ok: Logical value true if acquisition delay is OK to proceed with acquisition; false otherwise
%
%% NOTES
%   This function informs user that currently configured acquisition delay is too high for operation
%   while 'image striping' is enabled. Either the acquisition delay must be reduced or (more likely) the
%   user should disable image striping for this particular scan configuration.
%
%% CREDITS
%   Created 10/24/09, by Vijay Iyer
%% ******************************************

global state gh


%%%Handle startup case (calls via INI file callbacks at program startup)
if ~isfield(state.internal,'samplesPerLine')
    ok = true;
    return;
end

[startColumn endColumn] = determineAcqColumns();

ok = endColumn <= state.internal.samplesPerLine || state.acq.disableStriping || state.internal.numberOfStripes == 1;

if ok
    set(gh.configurationControls.etAcqDelay,'BackgroundColor',[1 1 1]);
else
    set(gh.configurationControls.etAcqDelay,'BackgroundColor',[1 0 0]);

    %Show warning dialog
    hDlg = warndlg({'Acquisition Delay is too high for image striping with scan as configured.'; ...
        'This can be fixed by either: ' ; ...
        '  1) selecting ''Disable Image Striping'' in the Configuration diaog, or'; ...
        '  2) reducing Acquisition Delay (if appropriate)'},...
        'Acq Delay Too High','modal');

    %Set position to just below current configurationControls position (whether visible or not)
    dlgPosn = getpixelposition(hDlg);
    configPosn = getpixelposition(gh.configurationControls.figure1);
    dlgPosn(1) = configPosn(1) + configPosn(3)/2 - dlgPosn(3)/2;
    dlgPosn(2) = configPosn(2)-dlgPosn(4)-20;
    setpixelposition(hDlg,dlgPosn);

    %Bind deleteFcn callback
    set(hDlg,'DeleteFcn',@closeWarnDlg);
end

    %Restore Acq Delay control color
    function closeWarnDlg(hObject,eventdata)
        set(gh.configurationControls.etAcqDelay,'BackgroundColor',[1 1 1]);
    end


end



