%% CHANGES
%   VI091608A: Handle case where saveDuringAcquisition is enabled, but autoSave is disabled -- Vijay Iyer 9/16/08
%   VI091608C: Accept optional input argument to allow this to be a 'direct' callback -- Vijay Iyer 9/16/08
%   VI091708A: Handle /all/ the control state updates here, based on autoSave state var. This is sometimes redundant with he INI-file callback, but that won't always be called -- Vijay Iyer 9/17/08
%   VI120408A: Handle improve mainControls GUI controls reflecting state of autoSave state var -- Vijay Iyer 12/04/08
%   VI121008A: Ensure that save/logging status is green-colored when either Logging is active or auto-save is enabled -- Vijay Iyer 12/10/08
%   VI011310A: The checkmark is now in the UserPreferencesGUI, not in the Main Controls Settings... menu -- Vijay Iyer 1/13/10
%   VI011310B: No longer give preemptive warning when KeepAllSlicesInMemory and AutoSave are both OFF. Defer this till a stack acquisition is initiated. -- Vijay Iyer 1/13/10
%   VI011310C: Duplicate control synchronization is now handled by INI file bindings and not needed here -- Vijay Iyer 1/13/10
%   VI092210A: Eliminate check of now defunct state.acq.saveDuringAcquisition -- Vijay Iyer 9/22/10
%
%% **********************************************
function updateAutoSaveCheckMark(h) %VI092210B %VI091608C
% BSMOD - 1/1/2 - sets check mark next to autoSave selection in settings menu

global gh state

%%%VI011310A: Removed%%%%%%
% get the index of the standard mode selection of the settings menu
% 	children=get(gh.mainControls.Settings, 'Children');
% 	index=getPullDownMenuIndex(gh.mainControls.Settings, 'Auto save');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

toggleableControls = [gh.mainControls.baseName gh.mainControls.fileCounter gh.mainControls.baseNameLabel gh.mainControls.fileCounterLabel];
checkboxControls = [gh.configurationControls.cbAutoSave gh.mainControls.cbAutoSave];

if state.files.autoSave==0 %VI092210A %VI091608A
    %set(children(index), 'Checked', 'off'); %VI091708A
	set(toggleableControls,'Enable','off');
	
    %%%VI011310B: Removed%%%%%%
    % 		if ~state.internal.keepAllSlicesInMemory
    % 			beep;
    % 			errordlg({ ...
    % 				'''Keep All Slices In Memory'' is OFF and ''Auto Save'' is OFF.' , ...
    % 				'Data will be lost for all acquisitions of more than 1 slice.', ...
    % 				'Recommend turning ''Auto Save'' on.'}, ...
    % 				'Warning', 0);
    %         end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    set(checkboxControls,'BackgroundColor',[1 0 0]); %VI120408A, VI121008A
else
    %set(children(index), 'Checked', 'on'); %VI091708A
    
	set(toggleableControls,'Enable','on');
	
    set(checkboxControls,'BackgroundColor',[0 .8 0]); %VI121008A
end

%%%VI011310C: Removed%%%%%%%%%
%     %%%%VI091708A%%%%%%%%
%     if state.files.autoSave
%         %set(gh.mainControls.cbAutoSave,'BackgroundColor',[0 .8 0]); %VI120408A
%         %set(gh.mainControls.stLogging,'BackgroundColor',[0 .8 0]); %VI120408A
%         set(gh.mainControls.cbAutoSave,'Value', 1);
%         %set(children(index), 'Checked', 'on'); %VI011310A: Removed
%     else
%         %set(gh.mainControls.cbAutoSave,'BackgroundColor',[1 0 0]); %VI120408A
%         %set(gh.mainControls.stLogging,'BackgroundColor',[1 0 0]); %VI120408A, VI121008A
%         set(gh.mainControls.cbAutoSave,'Value', 0);
%         %set(children(index), 'Checked', 'off'); %VI011310A: Removed
%     end
%     %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
