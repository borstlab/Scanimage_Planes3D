function out=savingInfoIsOK
%% CHANGES
%   updated by tp 10/25/01
%   VI11108A: Ensure that 'full file name' state variable gets updated following changes to baseName or save path
%   VI092410A: Actually use current directory when selecting 'Use Current' option; provide option to cancel -- Vijay Iyer 9/24/10

global state
st=[];
% TPMOD 2/6/02
if state.files.autoSave	
    if isempty(state.files.baseName) %if no base name is chosen
        disp('*** ERROR: Please enter a basename ***');
        answer  = inputdlg('Select base name','Choose Base Name for Acquisition',1,{state.files.baseName});
        if ~isempty(answer)
            state.files.baseName = answer{1};
            updateGUIByGlobal('state.files.baseName');
        else
            st='? Basename ';
        end
    end
    if isempty(state.files.savePath)
        disp('*** ERROR: Please set a save path using save ''File -> Set Save Path...'' ***');
        button = questdlg('A Save path has not been selected.','Do you wish to:','Select New Path','Use Current','Cancel','Select New Path');
        if strcmp(button,'Select New Path')
            setSavePath;
        elseif strcmp(button,'Use Current')  
            
            state.files.savePath = pwd(); %VI092410A
            updateGUIByGlobal('state.files.savePath');
        elseif strcmp(button,'Cancel') %VI092410A
            out = 0;
            return;
        end
    end
    if isempty(state.files.savePath)
        if isempty(st)
            st='? Save Path ';
        else
            st=[st ', Path'];
        end
    end
    if ~isempty(st)
        st=[st '?'];
        setStatusString(st);
        beep; %TPMOD 2/6/02
        out=0;
    else % everything is ok...
        updateFullFileName; %111108A: Do this every time there's been a successful update
        disp([clockToString(clock) ' *** '''  state.files.fullFileName ''' ***']);
        out=1;
    end 
else
    out=1;
end