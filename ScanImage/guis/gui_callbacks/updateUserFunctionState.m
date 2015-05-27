function updateUserFunctionState(mode,tableName,eventName,userFcnIndices)
%% function updateUserFunctionState(mode,tableName,eventName,userFcnIndices)
%  Load/store userFcns CFG-file state variables from/to the EventMap (which is not/cannot be directly encoded to/from CFG file) OR update listener object
%  
%
%% SYNTAX
%   updateUserFunctionState('updateListener',tableName,eventName)
%   updateUserFunctionState('updateListener',tableName,eventName,userFcnIndices)
%   updateUserFunctionSettings('storeStateVars',[],userFcnIndices)
%	updateUserFunctionSettings('loadStateVars',subsetInfix)
%
%       mode: One of {'loadStateVars' 'storeStateVars' 'updateListener'} indicating whether to load/store from/to CFG-file state vars
%		tableName: One of {'UserFcns' 'USROnlyFcns' 'OverrideFcns'} indicating which uitable is being modified.
%       eventName: <OPTIONAL> String cell array of eventNames, indicating which events to store state or update listener for
%       userFcnIndices: <OPTIONAL> Array of indices specifying which numbered userFcn to store state or update listener forupdate state/listene
%
%       NOTE: If eventName or userFcnIndices are not supplied, then /all/ events/userFcnIndices are stored/updated
%       NOTE: The eventName/userFcnIndices arguments are not supported for 'loadStateVars' mode -- this always occurs en-masse
%           
%% NOTES
%   Function exists to contain logic shared between UI callbacks and opencfg()
%
%   The update listener, load CFG, and store CFG operatins are combined into a single file, since they share some code -- which would spawn even more functions
%   Ideally, we might have made a ScanImage userFucntion class to combine this related functionality
%
%   NOTE: At this time, eventName MUST be left empty or omitted if using storeStateVars mode -- i.e. you must store all events for specified userFcnIdx
%   This could be fixed (i.e. insert/remove event info into current state var string -- but not worth effort at moment
%
%   NOTE: The constructUserFcnKernel() logic here is duplicated in the userFunctionsGUI(). This could/should be factored out.
%
%   TODO: Maybe consider a way to evaluate global vars passed to built-in functions -- Vijay Iyer 10/28/10
%       
%% CHANGES
%   VI102810A: Use standard (src,evnt) signature for standard callbacks, rather than custom arguments. scanimage event data can be passed via scanimage.EventManager.notifySmart(). However, don't pass event data for built-in functions -- Vijay Iyer 10/28/10
%
%% CREDITS
%   Created 10/1/10, by Vijay Iyer
%% *****************************************

global gh state

%Process input arguments
if nargin < 2 || isempty(tableName) || isempty(mode)
	error('Not enough input arguments.');
else
	if strcmp(mode,'loadStateVars')
		% we're sneaking the subset-name in via the 'tableName' var, so
		% make the assignment, and then set 'tableName' appropriately.
		subsetInfix = tableName;
		
		if strcmp(subsetInfix,'USR') || strcmp(subsetInfix,'CFG')
			tableName = 'UserFcns';
			saveTarget = subsetInfix;
 		elseif strcmp(subsetInfix,'USRONLY')
 			tableName = 'USROnlyFcns';
 			saveTarget = 'USR';
		elseif strfind(subsetInfix,'OVERRIDE')
			tableName = 'OverrideFcns';
			tokens = regexp(subsetInfix,'OVERRIDE(\w+)$','tokens');
			if ~isempty(tokens)
				saveTarget = tokens{1};
				saveTarget = saveTarget{1};
			else
				error('Invalid parameters.');
			end
		end
	end
	
	switch tableName
		case 'UserFcns'
			if strcmp(mode,'loadStateVars')
				hEvntMap = state.userFcns.(['hEventMap' subsetInfix]); 
				numReps = state.userFcns.maxNumUserFcns;
			else
				hEvntMap = state.userFcns.(['hEventMap' state.userFcns.saveTarget]);
				numReps = state.userFcns.maxNumUserFcns;
			end
 		case 'USROnlyFcns'
 			hEvntMap = state.userFcns.hEventMapUSRONLY;
 			numReps = 1;
		case 'OverrideFcns'
			if strcmp(mode,'loadStateVars')
				suffix = saveTarget;
			else
				suffix = state.userFcns.saveTarget;
			end
			hEvntMap = state.userFcns.(['hOverrideMap' suffix]);
			numReps = 1;
	end
end
	
if nargin < 3 || isempty(eventName)
	eventNames = hEvntMap.keys();
else
	eventNames = {eventName};
	
	if strcmp(mode,'loadStateVars')
		subsetInfix = eventName;
	end
end

if nargin < 4 || isempty(userFcnIndices)
	if strcmp(tableName,'UserFcns') && (strcmp(subsetInfix,'CFG') || strcmp(subsetInfix,'USR'))
		userFcnIndices = 1:state.userFcns.maxNumUserFcns;
	else
		userFcnIndices = 1;
	end
end

% Subset-dependent initialization
if strcmp(tableName,'UserFcns') || strcmp(tableName,'USROnlyFcns')
	structInit = struct(...
		'userFcnName','',...
		'userFcnListener',[],...
		'userFcnOptArgs','',...
		'userFcnKernel',[]);
elseif strcmp(tableName,'OverrideFcns')
	structInit = struct(...
		'userFcnName','',...
		'userFcnKernel',[]);
end

persistent isInitializedCFG;
persistent isInitializedUSR;
persistent isInitializedUSRONLY;
persistent isInitializedOVERRIDECFG;
persistent isInitializedOVERRIDEUSR;
if isempty(isInitializedCFG)
	isInitializedCFG = false;
	isInitializedUSR = false;
	isInitializedUSRONLY = false;
	isInitializedOVERRIDECFG = false;
	isInitializedOVERRIDEUSR = false;
	
	if ~isempty(state.userFcns.listenerArray)
		delete(state.userFcns.listenerArray);
		state.userFcns.listenerArray = [];
	end
end

switch mode
    
    case 'loadStateVars'
		if (~isInitializedCFG && strcmp(subsetInfix,'CFG')) || (~isInitializedUSR && strcmp(subsetInfix,'USR')) || (~isInitializedUSRONLY && strcmp(subsetInfix,'USRONLY')) ... 
				|| (~isInitializedOVERRIDECFG && strcmp(subsetInfix,'OVERRIDECFG')) || (~isInitializedOVERRIDEUSR && strcmp(subsetInfix,'OVERRIDEUSR'))
			%Clear all previous records/listeners
			structInit = repmat(structInit,numReps,1);

			for i=1:length(eventNames)
				hEvntMap(eventNames{i}) = structInit;
			end  
			
			switch subsetInfix
				case 'CFG'
					isInitializedCFG = true;
				case 'USR'
					isInitializedUSR = true;
				case 'USRONLY'
					isInitializedUSRONLY = true;
				case 'OVERRIDECFG'
					isInitializedOVERRIDECFG = true;
				case 'OVERRIDEUSR'
					isInitializedOVERRIDEUSR = true;
			end
				
		end
		
		if strcmp(tableName,'OverrideFcns')
			recordLength = 3;
		else
			recordLength = 4;
		end
       
		%Now loop through events for which there is actually information
		for i=1:numReps

			if strcmp(tableName,'UserFcns')
				suffix = num2str(i);
			else
				suffix = '';
			end
			usrFcnState = state.userFcns.(['userFcnBindings' subsetInfix suffix]);

			if ~isempty(usrFcnState)

				numEvents = length(usrFcnState)/recordLength;
				assert(round(numEvents)==numEvents);

				for j=1:numEvents
					eventName = usrFcnState{recordLength*(j-1)+1};

					if hEvntMap.isKey(eventName)

 						%Event name
 						eventStruct = hEvntMap(eventName);
% 						eventStruct(i).eventName = eventName; % DEQ20110308 - I don't think this needs to be here...

						%UserFcn name -- updates kernel and listener
						userFcnName = usrFcnState{recordLength*(j-1)+2};%%(recordLength-2)};
						eventStruct(i).userFcnName = userFcnName;
						eventStruct(i).userFcnKernel = constructUserFcnKernel(userFcnName); %Will be empty if userFcnName is empty or not a valid filename
							
						%Optional arguments
						if strcmp(tableName,'UserFcns') || strcmp(tableName,'USROnlyFcns')
							eventStruct(i).userFcnOptArgs = usrFcnState{recordLength*(j-1)+(recordLength-1)};
						end

						%Update Event Map
						hEvntMap(eventName) = eventStruct;

						%Update Listener
						if strcmp(tableName,'UserFcns') || strcmp(tableName,'USROnlyFcns')
							hListener = updateListener({eventName},i);
							
							%Update Enabled property
							if ~isempty(hListener)
								hListener.Enabled = logical(eval(usrFcnState{recordLength*(j-1)+(recordLength)}));
% 								state.userFcns.([lower(saveTarget) 'StateCache'])(i).(eventName) = true;
% 							else
% 								state.userFcns.([lower(saveTarget) 'StateCache'])(i).(eventName) = false;
							end
							
% 							% disable the listener if the current subset doesn't match the current 'save target'
% 							if ~strcmp(saveTarget,state.userFcns.saveTarget)
% 								hListener.Enabled = false;
% 							end
						elseif strcmp(tableName,'OverrideFcns')
							%Update Enabled property
							if ~isempty(eventStruct(i).userFcnKernel)
								isEnabled = logical(eval(usrFcnState{recordLength*(j-1)+(recordLength)}));
 								
								if strcmp(saveTarget,'USR')
									state.userFcns.overrideStateCacheUSR(eventName) = isEnabled;
									% if there's an existing CFG override in this position, don't enable this one
									isEnabled = isEnabled && ~state.userFcns.overrideStateCFG(eventName);
									state.userFcns.overrideStateUSR(eventName) = isEnabled;
								elseif strcmp(saveTarget,'CFG')
									state.userFcns.overrideStateCFG(eventName) = isEnabled;
								end
								if isEnabled
									state.hSI.registerOverrideFcn(eventName,eventStruct(i).userFcnKernel);
								end
							end
							
% 							if ~strcmp(saveTarget,state.userFcns.saveTarget)
% 								state.hSI.unregisterOverrideFcn(eventName);
% 							end
						end
					end

				end               
			else
				% 'usrFcnState' is empty, so we must be loading a CFG/USR that doesn't have user-functions for this index. Parse the
				% table and clear any conflicting data.
% 				events = hEvntMap.keys;
% 
% 				for eventName = events
% 					existingEventStruct = hEvntMap(eventName{1});
% 					if i > length(existingEventStruct)
% 						continue;
% 					end
% % 					existingSaveTarget = existingEventStruct(i).saveTarget;
% % 					if strcmp(existingSaveTarget,saveTarget)
% % 						hEvntMap(eventName{1}) = structInit;
% % 					end
% 				end
			end

			%Update the appropriate GUI display (i.e. table entries)
			if any(strfind(subsetInfix,'OVERRIDE')) || i == state.userFcns.currentUserFcnIdx
				updateUserFunctionsGUI(tableName,[],true);
			end

		end      
        
    case 'updateListener'
        updateListener(eventNames,userFcnIndices);
        
    case 'storeStateVars' %Store event data (in EventMap) to CFG/USR vars, updating Listener objects, as needed        
        updateStateVars(eventNames,userFcnIndices);

    otherwise
        assert(false);
end

return;

	function updateStateVars(eventNames,userFcnIndices)

		if strcmp(tableName,'OverrideFcns')
			recordLength = 3;
		else
			recordLength = 4;
		end
		offset = 1;

		for i=userFcnIndices
			stateVarCFG = {};
			stateVarUSR = {};

			for j=1:length(eventNames)
				stateVarVal = {};

				evntStruct = hEvntMap(eventNames{j});

				if ~isempty(evntStruct(i).userFcnName) || ...
					(isfield(evntStruct(i),'userFcnOptArgs') && ~isempty(evntStruct(i).userFcnOptArgs))

					stateVarVal{end+recordLength} = ''; %Allocate additional event/usrFcn binding entry

					%Store entry 'fields'
					offset = 1;
					stateVarVal{end-(recordLength-offset)} = eventNames{j}; %Store eventName
					offset = offset + 1;
					stateVarVal{end-(recordLength-offset)} = evntStruct(i).userFcnName; %Store userFcnName   
					offset = offset + 1;
					
					if ~strcmp(tableName,'OverrideFcns')
						optArgsString = evntStruct(i).userFcnOptArgs;  
						stateVarVal{end-(recordLength-offset)} = optArgsString;
						offset = offset + 1;
					end
					
					if ~isempty(evntStruct(i).userFcnName) %Store 'Enabled' flag
						if strcmp(tableName,'OverrideFcns')
							if strcmp(state.userFcns.saveTarget,'CFG')
								overrideState = state.userFcns.overrideStateCFG(eventNames{j});
							elseif strcmp(state.userFcns.saveTarget,'USR')
								overrideState = state.userFcns.overrideStateUSR(eventNames{j}) || state.userFcns.overrideStateCacheUSR(eventNames{j});
							end
							stateVarVal{end-(recordLength-offset)} = num2str(overrideState);
						else
							stateVarVal{end-(recordLength-offset)} = num2str(evntStruct(i).userFcnListener.Enabled);
						end
					else
						stateVarVal{end-(recordLength-offset)} = num2str(false);
					end

					% store to the appropriate state var
					if strcmp(state.userFcns.saveTarget,'CFG')
						stateVarCFG = [stateVarCFG stateVarVal];
					elseif strcmp(state.userFcns.saveTarget,'USR')
						stateVarUSR = [stateVarUSR stateVarVal];
					end
				end        
			end

			switch tableName
				case 'UserFcns'
					if strcmp(state.userFcns.saveTarget,'CFG')
						infix = 'CFG';
						suffix = num2str(i);
					elseif strcmp(state.userFcns.saveTarget,'USR')
						infix = 'USR';
						suffix = num2str(i);
					end
					eval(['state.userFcns.(''userFcnBindings' infix suffix ''') = stateVar' infix ';']);
				case 'USROnlyFcns'
					state.userFcns.userFcnBindingsUSRONLY = stateVarUSR;
				case 'OverrideFcns'
					state.userFcns.('userFcnBindingsOVERRIDECFG') = stateVarCFG;
					state.userFcns.('userFcnBindingsOVERRIDEUSR') = stateVarUSR;
			end
		end

		return;
	end

	function hListener = updateListener(eventNames,userFcnIndices)

		for i=1:length(eventNames)
			eventName = eventNames{i};
			evntStruct = hEvntMap(eventName);

			for userFcnIdx = userFcnIndices

				%Clear previous listener, if any
				hPrevListener = evntStruct(userFcnIdx).userFcnListener;
				if ~isempty(hPrevListener)
					delete(hPrevListener);
					evntStruct(userFcnIdx).userFcnListener = [];
				end

				userFcnKernel = evntStruct(userFcnIdx).userFcnKernel;
				if ~isempty(userFcnKernel)
					%Determine whether listener to update/create should be enabled
					if ~isempty(evntStruct(userFcnIdx).userFcnListener)
						enabledTF = evntStruct(userFcnIdx).userFcnListener.Enabled; %Retain Enabled state of previous listener
					else
						enabledTF = true; %If creating new, start enabled!
					end

					%Create/update the actual listener
					optArgs = evntStruct(userFcnIdx).userFcnOptArgs; %string representation of cell array of arguments to pass to listener
					if isempty(optArgs)
						optArgs = {};
					else
 						optArgs = eval(optArgs); %Actual cell array of arguments
					end

					builtIn = isempty(fileparts(evntStruct(userFcnIdx).userFcnName));
					if builtIn
						hListener = addlistener(state.hSI,eventName,@(src,evnt)userFcnKernel(optArgs{:})); %VI102810A: Don't pass src/evnt to built-in functions -- there aren't built-in functions meant to act as a callback
					else
						hListener = addlistener(state.hSI,eventName,@(src,evnt)userFcnKernel(evnt.EventName,evnt.scimData,optArgs{:})); %VI102810A: Use normal src/evnt arguments
					end
					hListener.Enabled = enabledTF;

					%Maintain list of all valid listener handles (allows for ease of deleting en masse)
					if isempty(state.userFcns.listenerArray)
						state.userFcns.listenerArray = hListener;
					else
						state.userFcns.listenerArray(end+1) = hListener;
					end

					%Update event structure with new/modified listener
					evntStruct(userFcnIdx).userFcnListener = hListener;            
				else
					hListener = [];
				end

				%Update EventMap with modified event record
				hEvntMap(eventName) = evntStruct;
			end
		end

		return;
	end

	function userFcnKernel = constructUserFcnKernel(fcnName)

		[p,f,e] = fileparts(fcnName);

		if isempty(f) || (~isempty(p) && ~exist(fcnName,'file')) %Handle contingencey where file saved to CFG no longer exists
			userFcnKernel = [];
			return;
		end

		if isempty(p) %Built-in
			userFcnKernel = str2func(f);
		else
			prevPath = addpath(p,'-begin','-frozen');
			userFcnKernel = str2func(f);
			path(prevPath);
		end    
		
% 		if strcmp(tableName,'OverrideFcns')
% 			state.hSI.registerOverrideFcn(eventName,userFcnKernel);
% 		end
		
		return;
	end
end





