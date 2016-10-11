function closeprogram(obj,program_id)
%CLOSEPROGRAM   - @progmanager method for closing a program.
%   CLOSEPROGRAM(obj,program_id) method for progmanager to close a program.
%   Must supply a progmanager object and a program_id.  
%
%   The program_id can be a: string program name, a program object, or a 
%   handle to a graphics object that is part of the program.  
%
% 	If the program is not added to the manager already, an error occurs.  
%
% See also ADDPROGRAM, STARTPROGRAM, PARSEPROGRAMID
% Changed:
%       4/6/04 Objects need to get cleaned up properly. -- Tim O'Connor TO040604b
%       4/6/04 Changed error messages from '@progmanager/startprogram' to '@progmanager/closeprogram'. -- Tim O'Connor TO040604c
%              Added ability to `stop` objects -- Tim O'Connor TO040604d - (see TO040604b).
%       Flip up/down because the main GUI should get closed last, and is gauranteed to be the first in the list. -- TnT 6/16/04 TNT061604b
%       11/4/04 - Added a try/catch for object deletion. -- Tim O'Connor TO110404a
%     TO093005A: Completely reworked menus to be much more useful. Made a programmanger submenu on file. -- Tim O'Connor 9/30/05
%     TO120905L: Don't check for programs to be saved, let that be decided elsewhere (if at all). -- Tim O'Connor 12/9/05
%     TO071906B: Clean up a few bugs when closing complex and interrelated programs (deleteFigsOnClose, deleteObjectsOnClose, stopObjectsOnClose). -- Tim O'Connor 7/19/06
global progmanagerglobal
done=0;

% Parse inputs.
if nargin < 2
    error(['@progmanager/closeprogram: requires 2 input variables, a progmanager and a program object, name, or handle.']);
else
    [program_name,program_obj]=parseProgramID(program_id);
end

if nargin < 2
    %TO040604c
    error(['@progmanager/closeprogram: requires 2 input variables.']);
end

if ~isfield(progmanagerglobal.programs,program_name)
    %TO040604c
    error(['@progmanager/closeprogram: program ' program_name ' does not exist.']);
else
    if ~checkSaveStatus(obj,program_name)
        return
    end
    %TO071906B: Print a message for each program being closed.
    fprintf(1, '%s - @progmanager/closeprogram: Closing ''%s''...\n', datestr(now), program_name);
    
    %Flip up/down because the main GUI should get closed last, and is gauranteed to be the first in the list. -- TnT 6/16/04 TNT061604b
    guinames= flipud(fieldnames(progmanagerglobal.programs.(program_name).guinames));
    
    for var_counter=1:length(guinames)
        %TO061504b - Add a generic close function to the lifecycle.
        try
            feval(progmanagerglobal.programs.(program_name).guinames.(guinames{var_counter}).funchandle, 'genericCloseFcn', ...
                progmanagerglobal.programs.(program_name).guinames.(guinames{var_counter}).fighandle, [], ...
                progmanagerglobal.programs.(program_name).guinames.(guinames{var_counter}).fighandle);    
        catch
            fprintf(2, '@progmanager/closeprogram: GUI s has a malfunctioning genericCloseFcn. Skipping. Error: %s\n', guinames{var_counter}, lasterr);
        end
        
        if ishandle(progmanagerglobal.programs.(program_name).guinames.(guinames{var_counter}).fighandle)
            delete(progmanagerglobal.programs.(program_name).guinames.(guinames{var_counter}).fighandle);   %Delete GUI figure.
        end
        % When closing, we should close additional figure handles if they
        % exist.  We will check for numeric arrays, and then see if they
        % are handles.  We can change these properties by setting the
        % program manager defaults using SETPROGMANAGERDEFAULTS.
        if progmanagerglobal.internal.deleteFigsOnClose | progmanagerglobal.internal.deleteObjectsOnClose | ...
                progmanagerglobal.internal.stopObjectsOnClose
            if isstruct(progmanagerglobal.programs.(program_name).(guinames{var_counter}).variables)
                temp = struct2cell(progmanagerglobal.programs.(program_name).(guinames{var_counter}).variables);
                %TO071906B - Moved deletion of figures below the stopping and deletion of objects, for better consistency during program shutdown (no invalid handles in objects). 
                %            Broke things out into separate loops, because these operations must be done in sequence.
                %            Wrapped each for loop with the option's conditional, which had previously been inside the single large loop.
                %            The object stop/delete should be okay to share a loop.
                %TO071906B
                if progmanagerglobal.internal.deleteObjectsOnClose || progmanagerglobal.internal.stopObjectsOnClose
                    for look_for_handles=1:length(temp)
                        %TO040604d - see TO040604b.
                        if isobject(temp{look_for_handles})
                            if ismethod(temp{look_for_handles}, 'stop')
                                stop(temp{look_for_handles});
                            end
                        end

                        %4/6/04 - Objects need to get cleaned up properly. -- Tim O'Connor TO040604b
                        if isobject(temp{look_for_handles})
                            if progmanagerglobal.internal.stopObjectsOnClose
                                %TO040604d - see TO040604b.
                                if ismethod(temp{look_for_handles}, 'stop')
                                    try
                                        stop(temp{look_for_handles});
                                    catch
                                        %TO071906B - Added an error message for stopping.
                                        warning('Some program objects of type ''%s'' in %s:%s may not have been properly stopped: %s', class(temp{look_for_handles}), program_name, guinames{var_counter}, lasterr);
                                    end
                                end
                            end
                            if progmanagerglobal.internal.deleteObjectsOnClose
                                %11/4/04 - Added this try/catch. -- Tim O'Connor TO110404a
                                try
                                    if ismethod(temp{look_for_handles}, 'delete')
                                        delete(temp{look_for_handles});
                                    end
                                catch
                                    warning('Some program objects of type ''%s'' in %s:%s may not have been properly deleted: %s', class(temp{look_for_handles}), program_name, guinames{var_counter}, getLastErrorStack);
                                end
                            end
                        end
                    end
                end
                %TO071906B - Totally reworked the deletion of figure handles, to be vectorized, since it apparently was poorly written originally.
                if progmanagerglobal.internal.deleteFigsOnClose
                    potentialHandles = [];

                    %TO071906B - Do not delete handles that point to other running programs.
                    if exist('programnameslist') ~= 1
                        programHandles = [];
                        programnameslist = fieldnames(progmanagerglobal.programs);
                        for i = 1 : length(programnameslist)
                            guinameslist = fieldnames(progmanagerglobal.programs.(programnameslist{i}).guinames);
                            for j = 1 : length(guinameslist)
                                programHandles(length(programHandles) + 1) = ...
                                    progmanagerglobal.programs.(programnameslist{i}).guinames.(guinameslist{j}).fighandle;
                            end
                        end
                    end

                    for look_for_handles=1:length(temp)
                        if ~isempty(temp{look_for_handles})
                            if isnumeric(temp{look_for_handles})
                                %Do not check large arrays (large is a relative term, of course).
                                if length(temp{look_for_handles}) < 10
                                    if any(ishandle(temp{look_for_handles}))
                                        temparr = temp{look_for_handles};
                                        if ~isempty(temp)
                                            potentialHandles(length(potentialHandles) + 1 : length(potentialHandles) + length(temparr)) = temparr(:);
                                        end
                                    end
                                end
                            end
                        end
                    end

                    potentialHandles = potentialHandles(find(potentialHandles ~= 0));
                    potentialHandles = potentialHandles(find(~ismember(potentialHandles, programHandles)));%TO071906B - Do not delete handles that point to other running programs.
                    potentialHandles = potentialHandles(find(ishandle(potentialHandles)));
                    if ~isempty(potentialHandles)
                        try

                            delete(potentialHandles);
                        catch
                            %TO071906B - Added an error message for stopping.
                            warning('Some assumed figure handles in %s:%s may not have been properly deleted: %s', program_name, guinames{var_counter}, lasterr);
                        end
                    end
                end
                %TO071906B - This was the old (and highly inefficient) implementation.
                %for look_for_handles=1:length(temp)
                %    if progmanagerglobal.internal.deleteFigsOnClose & isnumeric(temp{look_for_handles})
                %        if any(ishandle(temp{look_for_handles}))
                %            handles_to_delete=temp{look_for_handles};
                %            handles_to_delete(~ishandle(handles_to_delete))=[];
                %            handles_to_delete(~handles_to_delete)=[];
                %            %TO071906B - Do not delete handles that point to other running programs.
                %            programnames = fieldnames(progmanagerglobal.programs);
                %            for i = 1 : length(programnames)
                %                handles_to_delete(find(handles_to_delete == ...
                %                    progmanagerglobal.programs.(programnames{i}).guinames.(progmanagerglobal.programs.(programnames{i}).mainGUIname).fighandle)) = [];
                %            end
                %            delete(handles_to_delete);
                %        end
                %    end
                %end
            end
        end
    end
    progmanagerglobal.programs=rmfield(progmanagerglobal.programs,program_name);  % Remove all the variable references.
end

% if the progmanager display is on, update it.
if getProgmanagerDefaults(obj,'ProgmanagerDisplayOn')
    progmanagerdisp(obj);
end


%TO093005A
setWindowsMenuItems(obj);

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out=checkSaveStatus(obj,program_name);
% This function will execute a dialog box if the state of the program has
% changed since the last save and the user is closing the program.
out=1;
%TO120905L - Don't check for programs to be saved, let that be decided elsewhere (if at all). -- Tim O'Connor 12/9/05
% if getProgramProp(obj,program_name,'program_needs_saving')
%     beep;
%     button = questdlg(['Save Changes to ' getProgramProp(obj,program_name,'program_object_filename') ' ?'],['Program ' program_name ' Not Saved!'] ...
%         ,'Yes','No','Cancel','Yes');
%     if strcmpi(button,'yes')
%         % Save file.
%         if ~saveprogram(obj,program_name,getProgramProp(obj,program_name,'program_object_filename')) 
%             out=checkSaveStatus(obj,program_name);
%         end
%     elseif strcmpi(button,'cancel')
%         out=0;
%     end
% end