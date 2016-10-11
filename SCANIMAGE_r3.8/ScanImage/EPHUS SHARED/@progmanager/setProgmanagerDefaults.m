function varargout=setProgmanagerDefaults(prog_object,property,value)
% SETPROGMANAGERDEFAULTS   - @progmanager method sets the default settings for the program manager.
%   SETPROGMANAGERDEFAULTS(prog_object,property) sets the value specified property from the
%   program manager global array.  
%
%   Note that changing default properties will immediately change the way
%   the daqmanager handles programs.
% 
%   See PROGMANAGER for details on the default properties that can be set.
%
%   See also GETPROGMANAGERDEFAULTS, PROGMANAGERDISP.

% Changes:
% 	Tom Pologruto 3/5/04 (TP030504a): Put error checking in for setting, and removed it from
% 	progmanagerdisp.
% 	Tom Pologruto 3/5/04 (TP030504b): udpated to include internal_settable_fields. 


if nargin == 3
    eval(['global ' prog_object.name]);
%    start TP030504 mod...
    editable_props=[getProgmanagerDefaults(prog_object,'editable_fields') getProgmanagerDefaults(prog_object,'internal_editable_fields')];
    [ismem,location]=ismember(property,editable_props);
    if ~ismem | isempty(editable_props)
        error(['Cannot Edit Progmanager Property: ' property '. It is invalid or does not exist.']);
    end
    current_value=getProgmanagerDefaults(prog_object,property);
    min_val=[getProgmanagerDefaults(progmanager,'min_val') getProgmanagerDefaults(progmanager,'internal_min_val')];
    max_val=[getProgmanagerDefaults(progmanager,'max_val') getProgmanagerDefaults(progmanager,'internal_max_val')];
    if isnumeric(current_value) 
        if ~isnumeric(value)
            error(['Cannot Edit Progmanager Property: ' property ' must be a numeric.']);
        end
        value=max(value,min_val{location});
        value=min(value,max_val{location});
    elseif ischar(current_value) & ~ischar(value)
        error(['Cannot Edit Progmanager Property: ' property ' must be a string.']);
    end
    eval([prog_object.name '.internal.' property '=value;']);    
elseif nargin == 1 | nargin == 2
    eval(['global ' prog_object.name]);
    disp(' ');
    disp('All Program Manager Fields:');
    disp(' ');
    disp((eval('base',[prog_object.name '.internal'])));
    if nargout <= 1
        varargout{1}=fieldnames(eval('base',[prog_object.name '.internal']));
    else
        error('@progmanager/setProgmanagerDefaults: too many output arguments.');
    end
end
