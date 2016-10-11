function toggleUserFunctionsSaveTarget(~,~,~,doUpdateCheckBoxes)
	global state gh;
	
	if nargin < 4 || isempty(doUpdateCheckBoxes)
		doUpdateCheckBoxes = true;
	end
	
	% set the CFG/USR toggles to the appropriate state
	% NOTE: 'state.userFcns.saveTarget' is already updated to its new value before entering this function.
	if strcmp(state.userFcns.saveTarget,'CFG')
		if doUpdateCheckBoxes
			set(gh.userFunctionsGUI.tbCFG,'Value',true);
			set(gh.userFunctionsGUI.tbUSR,'Value',false);
		end
		
		set(gh.userFunctionsGUI.tblUSROnlyFcns,'Enable','off');
		
% 		set(gh.userFunctionsGUI.etUsrFcnIdx,'Enable','on');
% 		set(gh.userFunctionsGUI.pbInc,'Enable','on');
% 		set(gh.userFunctionsGUI.pbDec,'Enable','on');
		
% 		% disable all USR listeners/overrides
% 		disableEvents('USR');
% 		% enable all CFG listeners/overrides
% 		enableEvents('CFG');
		
	elseif strcmp(state.userFcns.saveTarget,'USR')
		if doUpdateCheckBoxes
			set(gh.userFunctionsGUI.tbUSR,'Value',true);
			set(gh.userFunctionsGUI.tbCFG,'Value',false);
		end
		
		set(gh.userFunctionsGUI.tblUSROnlyFcns,'Enable','on');
		
% 		set(gh.userFunctionsGUI.etUsrFcnIdx,'Enable','off');
% 		set(gh.userFunctionsGUI.pbInc,'Enable','off');
% 		set(gh.userFunctionsGUI.pbDec,'Enable','off');
		
% 		disableEvents('CFG');
% 		enableEvents('USR');
	end
	
	% refresh the data displayed in the uitables
	updateUserFunctionsGUI('UserFcns');
	updateUserFunctionsGUI('OverrideFcns');
end

% DEQ20110321 - these were implemented due to a misunderstanding of the USR/CFG pushbutton behavior,
% namely, that USR-events and CFG-events were mutually exclusive. These functions are now obviated, 
% but I'll leave them here for the moment...
%
% function disableEvents(mode)
% 	global state;
% 	
% 	if strcmp(mode,'USR')
% 		eventMapsToDisable = {'hEventMapUSR'	'hEventMapUSRONLY'	'hOverrideMapUSR'};
% 	elseif strcmp(mode,'CFG')
% 		eventMapsToDisable = {'hEventMapCFG' 'hOverrideMapCFG'};
% 	end
% 	tableNamesToUpdate = {'UserFcns'		'USROnlyFcns'		'OverrideFcns'};
% 	
% 	for i = 1:length(eventMapsToDisable)
% 		mapName = eventMapsToDisable{i};
% 		tableName = tableNamesToUpdate{i};
% 		
% 		if strcmp(mapName,'hEventMapCFG')
% 			numReps = state.userFcns.maxNumUserFcns;
% 		else
% 			numReps = 1;
% 		end
% 		
% 		userFcnMap = state.userFcns.(mapName);
% 		for eventName = userFcnMap.keys();
% 			eventName = eventName{1};
% 			eventStruct = userFcnMap(eventName);
% 			for j = 1:numReps
% 				if strfind(mapName,'hEventMap')
% 					% disable all listeners
% 					if ~isempty(eventStruct(j).userFcnListener) && eventStruct(j).userFcnListener.Enabled
% 						state.userFcns.([lower(mode) 'StateCache'])(j).(eventName) = true;
% 						eventStruct(j).userFcnListener.Enabled = false;
% 						updateUserFunctionsGUI(tableName,eventName);
% 					else
% 						state.userFcns.([lower(mode) 'StateCache'])(j).(eventName) = false;
% 					end
% 				elseif strfind(mapName,'hOverrideMap')
% 					% unregister all overrides
% 					if state.userFcns.hUserFcnManager.isFcnOverridden(eventName)
% 						state.userFcns.hUserFcnManager.unregisterOverrideFcn(eventName);
% 						state.userFcns.([lower(mode) 'StateCache'])(j).(eventName) = true;
% 					else
% 						state.userFcns.([lower(mode) 'StateCache'])(j).(eventName) = false;
% 					end
% 				end
% 			end
% 		end
% 	end	
% end
% 
% function enableEvents(mode)
% 	global state;
% 
% 	if strcmp(mode,'USR')
% 		eventMapsToEnable = {'hEventMapUSR'	'hEventMapUSRONLY'	'hOverrideMapUSR'};
% 	elseif strcmp(mode,'CFG')
% 		eventMapsToEnable = {'hEventMapCFG' 'hOverrideMapCFG'};
% 	end
% 	tableNamesToUpdate = {'UserFcns'		'USROnlyFcns'		'OverrideFcns'};
% 	
% 	for i = 1:length(eventMapsToEnable)
% 		mapName = eventMapsToEnable{i};
% 		tableName = tableNamesToUpdate{i};
% 		
% 		if strcmp(mapName,'hEventMapCFG')
% 			numReps = state.userFcns.maxNumUserFcns;
% 		else
% 			numReps = 1;
% 		end
% 		
% 		userFcnMap = state.userFcns.(mapName);
% 		for eventName = userFcnMap.keys();
% 			eventName = eventName{1};
% 			eventStruct = userFcnMap(eventName);
% 			for j = 1:numReps
% 				if strfind(mapName,'hEventMap')
% 					% enable all listeners
% 					if ~isempty(eventStruct(j).userFcnListener) && state.userFcns.([lower(mode) 'StateCache'])(j).(eventName) == true
% 						eventStruct(j).userFcnListener.Enabled = true;
% 						updateUserFunctionsGUI(tableName,eventName);
% 					else
% 						state.userFcns.([lower(mode) 'StateCache'])(j).(eventName) = false;
% 					end
% 				elseif strfind(mapName,'hOverrideMap')
% 					% register all overrides
% 					if ~isempty(eventStruct(j).userFcnKernel) && state.userFcns.([lower(mode) 'StateCache'])(j).(eventName)
% 						state.userFcns.hUserFcnManager.registerOverrideFcn(eventName,eventStruct(j).userFcnKernel);
% 					end
% 				end
% 			end
% 		end
% 	end	
% end