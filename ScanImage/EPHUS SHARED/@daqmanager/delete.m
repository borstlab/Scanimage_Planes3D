function success = delete(dm)
%% function success = delete(dm)
%     Delete a daqmanager object.
%%
%%  WARNING: This will invalidate all pointers to this object. Yet,
%%           the pointers will still exist, and possibly appear valid.
%%
%%  SUCCESS = delete(OBJ)
%%
%%  SUCCESS == 1 if the delete succeeded, 0 if there was a problem.
%% 
%%  Created - Tim O'Connor 11/27/04
%
%%  MODIFICATIONS
%    1/27/05 Tim O'Connor (TO012705a): Delete the analoginput/analogoutput objects.
%    8/26/05 Tim O'Connor (TO082605A): Changed 'disable', which is undefined, to 'disableChannel'. 
%                                      Confirm global deletion.
%    8/29/06 Tim O'Connor (TO082905A): Wrapped the `stop` calls in try-catch blocks. Added loop iteration over the low-level objects.
%    2/23/08 Vijay Iyer (VI022308A): Only stop AO objects if they're present!
%    2/23/08 Vijay Iyer (VI022308B): Really clear the global structure this time...
%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%

global gdm;

success = 0;

for i = 1 : length(gdm(dm.ptr).channels)
    %Stop any running channels.
    if gdm(dm.ptr).channels(i).state == 2
        stopChannelImmediate(dm, gdm(dm.ptr).channels(i).name);
    end

    %Disable the channels.
    disableChannel(dm, gdm(dm.ptr).channels(i).name);%TO082605A
end

%TO012705a
%TO082905A
if ~isempty(gdm(dm.ptr).aos)
    try
        for i = 1 : length(gdm(dm.ptr).aos)
            if ~isempty(gdm(dm.ptr).aos{i}) %VI022308A
                stop(gdm(dm.ptr).aos{i});
            end
        end
    catch
        gdm(dm.ptr).aos{i}
        warning('Failed to stop analogoutput objects before deletion.');
    end
end
success = 0;
for i = 1 : length(gdm(dm.ptr).aos)
    try
        if ~isempty(gdm(dm.ptr).aos{i})
            delete(gdm(dm.ptr).aos{i});
        end
    catch
        warning('Failed to properly delete analogoutput object ''%s''', get(gdm(dm.ptr).aos{i}, 'Name'));
    end
end
success = 1;
if ~isempty(gdm(dm.ptr).ais)
    try
        for i = 1 : length(gdm(dm.ptr).ais)
            if ~isempty(gdm(dm.ptr).ais{i})
                stop(gdm(dm.ptr).ais{i});
            end
        end
    catch
        warning('Failed to stop analoginput objects before deletion.');
    end
end
success = 0;
for i = 1 : length(gdm(dm.ptr).ais)
    try
        delete(gdm(dm.ptr).ais{i});
    catch
        warning('Failed to properly delete analoginput object ''%s''', get(gdm(dm.ptr).ais{i}, 'Name'));
    end
end
success = 1;

gdm(dm.ptr).adaptor = '';
gdm(dm.ptr).aos = [];
gdm(dm.ptr).ais = [];%TO012705a
gdm(dm.ptr).channels = [];

%Delete the associated @callbackmanager
delete(gdm(dm.ptr).cbm);

%Get rid of the global structure, if possible.
if length(gdm) == 1
    gdm = [];%TO082605A
    clear global gdm; %VI022308B
end

success = 1;

return;