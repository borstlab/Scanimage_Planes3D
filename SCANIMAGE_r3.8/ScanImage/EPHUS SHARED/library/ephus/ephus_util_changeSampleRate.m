function [ ] = ephus_util_changeSampleRate( sampleRate )
%EPHUS_UTIL_CHANGESAMPLERATE
%   Changes the sampleRate in all (relevant) programs. 
%
%   NOTE: Individual programs save sampleRate to .settings files (hotswitches and configurations). Loading .settings files may overwrite
%   the sampleRate. If you wish to change the sampleRate *globally*, you should recreate all hotswitches and configurations, 
%   or alternatively iterate through each hotswitch/configuration, first calling this function to change the sampleRate, 
%   and then resaving the relevant programs for the hotswitch/configuration.
%
% 2011-10-03 -- Tim O'Connor and Ben Suter
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

progs = { 'ephys', 'stimulator', 'acquirer', 'mapper' };

for k=1:numel(progs)
    if isstarted(progmanager, progs{k})
        setGlobal(progmanager, 'sampleRate', progs{k}, progs{k}, sampleRate);
    end
end

autonotes_addNote(['Sampling rate changed to ' num2str(sampleRate)]);
