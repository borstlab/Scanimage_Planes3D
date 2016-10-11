function val = getCycleVar(varName,iteration)
%GETCYCLEVAR Returns the value of the given cycle var, using the current cycle iteration (or optionally, via an argument).
global state;

if nargin < 2 || isempty(iteration)
	iteration = state.cycle.iteration;
end

if isfield(state.cycle.cycleTableStruct,varName) && iteration <= length(state.cycle.cycleTableStruct)
	val = state.cycle.cycleTableStruct(iteration).(varName);
	
	if isempty(val)
		val = [];
		return;
	end
	% determine and return the appropriate type
	if isnumeric(val)
		return;
	else
		% test for a vector
		if regexp(val,'^\[.*\]$')
			tokens = regexpi(val,'([\-0-9\.]+|NaN)*','tokens');
			if ~isempty(tokens)
				val = [];
				for i=1:length(tokens)
					token = str2double(tokens{i});
					val = [val token];
				end
            end
        elseif ~isnan(str2double(val))
            val = str2double(val);
		else
			% val is a string, do nothing
		end
	end
elseif strcmp(varName,'configPath')
	val = state.cycle.cycleConfigPaths{iteration};
else
	val = [];
	return;
end
end