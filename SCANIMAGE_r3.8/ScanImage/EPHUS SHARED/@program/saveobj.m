function obj_out = saveobj(obj)
%SAVEOBJ   - @program save overloaded function.
%  SAVEOBJ(obj) will run the save script prior to writing the object to
%  disk.
%
%   This function will look at the program manager for values that pertain to
%   the program being saved, and the current values will be stored in the
%   program object.  The following are saved in the object:
% 		variables whose ConfigFlag is set the correct bit for saving based on
% 		the Program manager default property ConfigBitForSaving
% 		
% 		figure position and visible properties.
%
%  See also LOADOBJ

progmanager_obj=progmanager;    % reference to program manager object.
if isprogram(progmanager_obj,obj.program_name)
    % Remember current versions of software when saving object.
    [obj.version, obj.progmanager_version]=getProgramVersion(progmanager_obj,obj);
    aliases=fieldnames(obj.aliases);
    config_flag=getProgmanagerDefaults(progmanager_obj,'ConfigBitForSaving');
    for alias_counter=1:length(aliases)
        % Write variables.
        obj.aliases.(aliases{alias_counter}).variables=getVarWithConfigFlag(progmanager_obj,obj.program_name,aliases{alias_counter},config_flag);
        % Remember fig properties.
        fig_handle=getHandleFromName(progmanager_obj,aliases{alias_counter},obj.program_name);
        obj.aliases.(aliases{alias_counter}).fig_props.Position=get(fig_handle,'Position');
        obj.aliases.(aliases{alias_counter}).fig_props.Visible=get(fig_handle,'Visible');
    end
end
obj_out=obj;

