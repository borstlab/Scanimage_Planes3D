function done=parseGlobalCellArray(prog_object,gui_fig_handle)
%PARSEGLOBALCELLARRAY   - Program Manager Variable Parser.
%   PARSEGLOBALCELLARRAY takes the output of the GUI sub function called makeGlobalCellArray and will
%   update the program manager variable accordingly.  The program manager
%   structure is preconfigured using the STARTPROGRAM method for the
%   program manager object.
%
%   Output will be 1 if successful, and 0 otherwise.
%
%   The output of the makeGlobalCellArray subfunction in each GUI is a cell
%   array or parameter/value paris, simialr to varargin in generic MATLAB
%   programming.  This cell array has all the odd numbered elements as
%   strings which specify the parameter being set.  The value of the
%   parameter is in the adjacent element (the even elements).
%
%   The parser will know that a new variable is being defined when the
%   parameter is unrecognized by the parser.  All elements between variable
%   names are flags for that variable.  These flags include:
%
% 		gui:        [char] specifying the GUI to tie this variable to.
% 		config:     [integer] specifies if the variable is part of the header or part fo the configuration for this program.
%                   
% 					A binary system is used.  The least significant bit is the flag for being part of the configuration file,
%                   while the second most significant bit tells if the variable is part of the header. -- This statement is no longer true (TO112805A).
%
%                   Bit flag definitions (big-endian):
%                      order  binary   meaning
%                      -----  ------   -------
%                       1     (0001) - configuration
%                       2     (0010) - header
%                       4     (0100) - lightweight configuration
%                       >4    (....) - reserved for future use
%                     
% 					For example,
%                       config 7 (0111) would imply that the variable is part of the lightweight configuration, header, and the configuration.
%                       config 6 (0110) would imply that the variable is part of the lightweight configuration and header, but NOT the configuration.
%                       config 5 (0101) would imply that the variable is part of the lightweight configuration and configuration, but NOT the header.
%                       config 4 (0100) would imply that the variable is part of the lightweight configuration (miniSettings as per TO062306D).
% 						config 3 (0011) would imply that the variable is part of both the header and configuration.
% 						config 2 (0010) would imply that the variable is part of the header and but NOT the configuration.
% 						config 1 (0001) would imply that the variable is part of the configuration but NOT the header.
%                       config 0 (0000) would imply that the variable is not saved anywhere.
%
% 					Note that these flags are also used by PROGRAM objects
% 					when they are being saved.  See the PROGMANAGER
% 					settings for details on how to interpret the config
% 					flags.
%
% 		numeric:    [bool] DEPRECATED: use class instead. 
% 		class:      [char]  can be 'char', 'numeric','double','int',or 'bool'  specifies the class of the variable.  'char' is default.          
% 		min:        [double] specifies minimum value of variable if it is numeric.  Ignored otherwise.
% 		max:        [double] specifies maximum value of variable if it is numeric.  Ignored otherwise.
%
%
%   See also STARTPROGRAM, ADDPROGRAM, PROGMANAGER, PARSESTRUCTSTRING, PROGRAM
%
% Changes
%    % TPMOD031704a: Fix Edit Box Setting problem of Min and Max
%      TO040804a: Check for an badly shaped array and fail immediately. -- Tim O'Connor 4/8/04
%      TO101304a: Watch out for the 'guidata(gcbo)' portion of the callback, which returns []. -- Timothy O'Connor 10/13/04
%      TO122204a: Automatically set the hObject variable into the program. -- Tim O'Connor 12/22/04
%      TO122304a: Created an error message, instead of simply doing a return. -- Tim O'Connor 12/23/04
%      TO112805A: Updated the config flags documentation to reflect the preferred usage. -- Tim O'Connor 11/28/05
%      TO113005B: Warn when an illegal config flag is encountered. -- Tim O'Connor 11/30/05
%      TO120105B: The config flag must default to 0 for all variables (especially when it is left undefined in the program). -- Tim O'Connor 12/1/05
%      TO120105C: This whole function's a total mess and should be rewritten from scratch. For now, just make sure the variable name is initialized. -- Tim O'Connor 12/1/05
%      TO120705C: Inserted a check and warning for invalid GUI parameter cases (it's case-sensitive, apparently). -- Tim O'Connor 12/7/05
%      TO062306D: Created a lightweight configuration (miniSettings), mainly for use in cycles. Only important run-time variables should get this value. -- Tim O'Connor 6/23/06
%      VI072808A: Handle 2008b callback syntax compatibility issue -- Vijay Iyer 7/28/08
done=0;
global progmanagerglobal
validflags={'gui','config','numeric','class','min','max'};
UserData=get(gui_fig_handle,'UserData');
gui_name=UserData.guiname; % gui name.
program_name=UserData.progname; %  program name.
try
% fprintf(1, 'experimentSavingGui: gui_fig_handle = %8.11f\n', gui_fig_handle);
% get(gui_fig_handle)
    globalarray=feval(progmanagerglobal.programs.(program_name).guinames.(gui_name).funchandle,'makeGlobalCellArray',gui_fig_handle,[],0);
    globalarray = cat(2, globalarray, {'hObject', gui_fig_handle, 'Class', 'figure_handle', 'Config', 0});%TO122204a, TO120105C
catch
    warning('Failed to retrieve globalCellArray from %s - %s: %s', program_name, gui_name, lasterr);%TO122304a
    return;
end

%4/8/04 Check for an badly shaped array and fail immediately. -- Tim O'Connor TO040804a
if mod(prod(size(globalarray)), 2) ~= 0
    error('Global array for program `%s-%s` must contain an even number of elements.', program_name, gui_name);
end

globalarray(end+1:end+2)={'done',''};
assignin('base','globalarray',globalarray);
counter=1;
value_assigned=1;
currentvariablename = 'UNINITIALIZED';%TO12105C: This whole function's a total mess and should be rewritten from scratch. For now, just make sure this is initialized. -- Tim O'Connor 12/1/05

while counter <= length(globalarray)
    param=globalarray{counter};
    if ~ischar(param)
        error('progmanagerglobal: invalid output from makeGlobalCellArray subfunction in GUI ''%s'' for variable ''%s''.', ...
            gui_name, currentvariablename);
    end
    value=globalarray{counter+1};
    counter=counter+2;
    if strcmpi(lower(param),'gui')
        [object_name,gui_name_gui,prog_name_gui]=parseStructString(value);
        if isempty(prog_name_gui)
            prog_name_gui=program_name;
        end
        if isempty(gui_name_gui)
            gui_name_gui=gui_name;
        end
        if isempty(object_name)
            warning(['parseGlobalCellArray: Bad Gui ' value  '  for ' param '. Skipping.']); 
        else
            fullGuiName={[prog_name_gui '.' gui_name_gui '.' object_name]};
            if isfield(progmanagerglobal.programs.(program_name).(gui_name).variableGUIs,currentvariablename)
                progmanagerglobal.programs.(program_name).(gui_name).variableGUIs.(currentvariablename)=...
                    [progmanagerglobal.programs.(program_name).(gui_name).variableGUIs.(currentvariablename) fullGuiName];
            else
                progmanagerglobal.programs.(program_name).(gui_name).variableGUIs.(currentvariablename)=fullGuiName;
            end
            if isfield(GUI_UserData, param) & iscell(GUI_UserData.(param))
                GUI_UserData.(param)=[GUI_UserData.(param) fullGuiName];
            else
                GUI_UserData.(param)=fullGuiName;
            end
            value_assigned=0;
        end
    elseif any(strcmpi(lower(param), {'class','min','max'}))
        GUI_UserData.(param)=value;
        value_assigned=0;
    elseif  strcmpi(lower(param),'config')
        progmanagerglobal.programs.(program_name).(gui_name).configflags.(currentvariablename)=value;
        value_assigned=0;
        %TO113005B - Warn when an illegal config flag is encountered.
        %TO062306D - The maximum value is now 7, also watch out for negatives.
        if value > 7 | value < 0
            fprintf(2, 'Warning: Invalid variable config flag - ''%s:%s:%s'': %s\n', program_name, gui_name, currentvariablename, num2str(value));
        end
    else
        if ~value_assigned & ~isempty(GUI_UserData)
            % Configure GUI Uicontrol object with parameters set.
            if isfield(GUI_UserData,'Gui')  % There is a GUI tied to this variable...
                % Check and warn about redundant GUIs being tied to the
                % same variable.
                [uniqueGUINames]=unique(GUI_UserData.Gui);
                if length(GUI_UserData.Gui) ~= length(uniqueGUINames)
                    disp([mfilename ': redundant GUIs for variable ' currentvariablename '. Check GUI for correct GUI Tags.']);
                end
                for guicounter = 1:length(progmanagerglobal.programs.(program_name).(gui_name).variableGUIs.(currentvariablename))
                    [object_name,gui_name_gui,prog_name_gui]=parseStructString(progmanagerglobal.programs.(program_name).(gui_name).variableGUIs.(currentvariablename){guicounter});
                    uicontrol_object=progmanagerglobal.programs.(prog_name_gui).(gui_name_gui).guihandles.(object_name);
                    GUI_UserData.variable=[prog_name_gui '.' gui_name_gui '.' currentvariablename];
                    currentcallback=get(uicontrol_object,'Callback');
                    
                    if verLessThan('matlab','7.7')  %VI072808A - Handle 2008b compatibility issues
                        stringCallback = true; %the callback was specified as a string in GUIDE
                    else
                        if isa(currentcallback,'function_handle')
                            stringCallback = false;
                            
                            currentcallback = func2str(currentcallback);
                            idx = strfind(lower(currentcallback), '@(hobject,eventdata)');
                            if ~isempty(idx)
                                currentcallback = currentcallback(idx+20:end);
                            end
                            currentcallback = strrep(currentcallback, 'hObject', 'getParent(gcbo, ''figure'')'); %I'm not sure why 'gcbo' is not sufficient (VI072808)
                            currentcallback = strrep(currentcallback, 'guidata', 'guihandles'); %I'm not sure why this is needed (VI072808)
                            currentcallback = strrep(currentcallback, 'eventdata', '[]');                      
                        else
                            stringCallback = true;  %the callback was specified as a string in GUIDE
                        end                            
                    end
                    if stringCallback %VI072808A 
                        %TO101304a - Watch out for the 'guidata(gcbo)' portion of the callback, which now returns []. - Timothy O'Connor 10/13/04
                        currentcallback = strrep(currentcallback, 'guidata(gcbo)', 'guihandles(getParent(gcbo, ''figure''))');
                    end
                    newcallback=['updateVariableFromGUI(progmanager,gcbo),' currentcallback ];

                    % TPMOD031704a: Fix Edit Box Setting problem
                    style=get(uicontrol_object,'style');
                    if any(strcmpi(style, {'popupmenu', 'listbox', 'slider'}))
                        if isfield(GUI_UserData,'Min') 
                            set(uicontrol_object,'Min',GUI_UserData.Min);
                            if ~isfield(GUI_UserData,'Max')
                                set(uicontrol_object,'Max',2^16-1);
                                GUI_UserData.Max=get(uicontrol_object,'Max');
                            end
                        end
                        if isfield(GUI_UserData,'Max') 
                            set(uicontrol_object,'Max',GUI_UserData.Max);
                            if ~isfield(GUI_UserData,'Min')
                                set(uicontrol_object,'Min',-2^16-1);
                                GUI_UserData.Min=get(uicontrol_object,'Min');
                            end
                        end
                    end
                    if strcmpi('slider',get(uicontrol_object,'style')) & isfield(GUI_UserData,'Class') & ...
                            strcmpi(GUI_UserData.Class,'int') %If this is a slide for integers, set it as such.
                        stepsize=1/(get(uicontrol_object,'Max')-get(uicontrol_object,'Min'));
                        set(uicontrol_object,'SliderStep',[stepsize stepsize]);
                    end
                    set(uicontrol_object,'UserData',GUI_UserData,'Callback',newcallback);
                    setGUIValue(prog_object,uicontrol_object,currentvariablevalue);
                end
            else
                %TO120705C: This can be an extremely painful error to debug. Make it fail-fast. -- Tim O'Connor 12/7/05
                if ismember('gui', lower(fieldnames(GUI_UserData)))
                    fprintf(2, 'Warning: An invalid case for the case-sensitive ''Gui'' parameter name may have been detected in the global cell array for the %s:%s::%s variable.\n         This must be corrected to tie this variable to its gui(s) properly.\n%s\n', ...
                        program_name, gui_name, currentvariablename, getStackTraceString);
                end
            end
        end
        % here, if the varible is already assigned a value from the program
        % object, then do not overwrite that value, but leave it.  If there
        % is no value, then use the default from the GUI default.
        if ~isfield(progmanagerglobal.programs.(program_name).(gui_name).variables,param)
            progmanagerglobal.programs.(program_name).(gui_name).variables.(param)=value;
        end
        currentvariablename=param;
        progmanagerglobal.programs.(program_name).(gui_name).configflags.(currentvariablename) = 0;%TO120105B: This must default to 0. -- Tim O'Connor 12/1/05
        currentvariablevalue=progmanagerglobal.programs.(program_name).(gui_name).variables.(param);
        
        GUI_UserData=struct;
        GUI_UserData.lastvalue=[];
        value_assigned=1;    
    end
end
done=1;