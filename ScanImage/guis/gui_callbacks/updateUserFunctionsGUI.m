function updateUserFunctionsGUI(tableName,evntName,forceUpdate)
%% function updateUserFunctionsGUI(evntName)
%  Function to update display of  User Functions data (i.e. the data table), either of specified event, or all events
%
%% SYNTAX
%   updateUserFunctionsGUIDisplay(tableName,evntName)
%   updateUserFunctionsGUIDisplay(evntName,forceUpdate)
%	updateUserFunctionsGUIDisplay(hTable,forceUpdate)
%
%		tableName: One of {'UserFcns' 'USROnlyFcns' 'OverrideFcns'} indicating which uitable to update.
%       evntName: Name of event for which to update table data (i.e. the row to update)    
%		forceUpdate: <LOGICAL - Default=false> If true, GUI update occurs even if figure is not visible
%   
%% NOTES
%   Function exists to contain logic shared between UI callbacks and opencfg()
%
%% CREDITS
%   Created 9/30/10, by Vijay Iyer
%% *****************************************

global gh state

if nargin < 3 || isempty(forceUpdate)
    forceUpdate = false;
end

if nargin < 2 || isempty(evntName)
	allEvents = true;
	evntName = '';
else
	allEvents = false;
end
	
if nargin < 1
	error('Not enough input arguments.');
end

switch tableName
	case 'UserFcns'
		table = gh.userFunctionsGUI.tblUserFcns;
		hEvntMap = state.userFcns.(['hEventMap' state.userFcns.saveTarget]);
		userFcnIdx = state.userFcns.currentUserFcnIdx;
	case 'USROnlyFcns'
		table = gh.userFunctionsGUI.tblUSROnlyFcns;
		hEvntMap = state.userFcns.hEventMapUSRONLY;
		userFcnIdx = 1;
	case 'OverrideFcns'
		table = gh.userFunctionsGUI.tblOverrideFcns;
		hEvntMap = state.userFcns.(['hOverrideMap' state.userFcns.saveTarget]);
		userFcnIdx = 1;
end

if forceUpdate || strcmpi(get(gh.userFunctionsGUI.figure1,'Visible'),'on') %Don't bother with updating if the GUI is not visible.
	if nargin > 0 && (hEvntMap.isKey(evntName) || allEvents)
        tblData = get(table,'Data');
        
        eventNamesOrdered = tblData(:,1);

        if allEvents %update /all/ events (rows)        
            for rowIdx=1:length(eventNamesOrdered)            
                updateRowData(rowIdx,eventNamesOrdered{rowIdx});
            end
        else %Update selected event               
            [~,rowIdx] = ismember(evntName,eventNamesOrdered);
            updateRowData(rowIdx,evntName);            
        end    

        %Actually update the table
        set(table,'Data',tblData);
	end
end

    function updateRowData(rowIdx,selEvnt)
		evntStruct = hEvntMap(selEvnt);
		
        if ~strcmp(tableName,'OverrideFcns')
            %UsrFcn Name
            userFcnName = evntStruct(userFcnIdx).userFcnName;
            if isempty(userFcnName)
                tblData{rowIdx,2} = '';
            else
                [~,fNameNoExt] = fileparts(userFcnName);
                tblData{rowIdx,2} = fNameNoExt;
            end

            %Optional Arguments  
            tblData{rowIdx,3} = evntStruct(userFcnIdx).userFcnOptArgs;

            %Enabled Flag
			if isempty(evntStruct(userFcnIdx).userFcnListener)
                tblData{rowIdx,4} = false;
            else
                tblData{rowIdx,4} = evntStruct(userFcnIdx).userFcnListener.Enabled;
			end
		elseif strcmp(tableName,'OverrideFcns')
            %overrideFcnName 
            userFcnName = evntStruct.userFcnName;
            if isempty(userFcnName)
                tblData{rowIdx,2} = '';
            else
                [~,fNameNoExt] = fileparts(userFcnName);
                tblData{rowIdx,2} = fNameNoExt;
            end
            
            %Enabled flag
			if ~isempty(evntStruct(userFcnIdx).userFcnKernel) && (state.userFcns.(['overrideState' state.userFcns.saveTarget])(selEvnt) || ...
					(strcmp(state.userFcns.saveTarget,'USR') && state.userFcns.(['overrideStateCache' state.userFcns.saveTarget])(selEvnt)))
                tblData{rowIdx,3} = true;
            else
                tblData{rowIdx,3} = false;
			end
        end
    end
end