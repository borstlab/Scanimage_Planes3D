function savedefaults
%SAVEDEFAULTS   - save default program manager files to disk.
%  SAVEDEFAULTS saves the default fields of the
%  program manager object to disk in filename.mat file in the same
%  directory as the progmanager function.
%
filename=getProgmanagerDefaults(progmanager,'filename');
names=getProgmanagerDefaults(progmanager,'editable_fields');
for counter=1:length(names)
    eval([names{counter} '=getProgmanagerDefaults(progmanager,names{counter});']);
    % Remove periods from structure cals before saving.
    periods=strfind(names{counter},'.');
    if ~isempty(periods)
        names{counter}=names{counter}(1:periods(1)-1);
    end
end
save(filename,names{:});      
