function obj_out=editprogram(obj)
%EDITPROGRAM   - edit program object graphically.
% EDITPROGRAM(obj) is a method of the program class, which allows the user
% to update fields of the  object including the mainGUI Name and the GUI
% aliases and mfile names.
%
% See also PROGMANAGERDISP
obj_out=obj;
objstructure=get(obj);
editablefields=objstructure.editable_fields;
min_val=objstructure.min_val;
max_val=objstructure.max_val;
param={};
value={};
full_struct_name={};
for fncounter=1:length(editablefields)
    if isstruct(objstructure.(editablefields{fncounter}))
        struct_name=editablefields{fncounter};
        f_names=fieldnames(objstructure.(struct_name));
        for fname_counter=1:length(f_names)
            param=[param  {['Alias ' num2str(fname_counter)],['M-File ' num2str(fname_counter)]}];
            value=[value {[f_names{fname_counter}]} {objstructure.(struct_name).(f_names{fname_counter}).m_filename}];
            full_struct_name=[full_struct_name {[struct_name '.' f_names{fname_counter}],[struct_name '.' f_names{fname_counter} '.m_filename']}];
        end
    else
        param=[param editablefields(fncounter)];
        value=[value {objstructure.(editablefields{fncounter})}];
        full_struct_name=[full_struct_name editablefields(fncounter)];
    end
end
% check to make sure we have something to edit.
if isempty(param)
    beep,disp(['No Editable Properties for Program ' obj.program_name]),return;
end

% Open the dialog box
answer=genericPropertyEditor(['Edit ' obj.program_name],param,value);

if ~isempty(answer)
    % Update the program with any updates.
    for param_counter=1:length(param)
        if ~isequal(value{param_counter},answer{param_counter})
            [last,second,first]=parseStructString(full_struct_name{param_counter})
            if isempty(second) & isempty(first)
                obj_out.(last)=answer{param_counter};
            elseif isempty(first)
                obj_out.(second).(answer{param_counter})= obj_out.(second).(last);
                obj_out.(second)=rmfield(obj_out.(second),last);
            else
                obj_out.(first).(second).(last)=answer{param_counter};
            end
        end
    end
end