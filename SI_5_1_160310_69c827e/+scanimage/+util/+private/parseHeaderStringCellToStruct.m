function [s] = parseHeaderStringCellToStruct(frameStringCell)
%   NOTE:   We are not using vernInfo from now on. From now on we will assume that every 
%           expression found in the TIFF header
%         from now on we will
%
    numImg = numel(frameStringCell);
    s = cell(numImg,1);

    for frameIdx = 1:numImg
        s{frameIdx,1} = struct();
        rows = textscan(frameStringCell{frameIdx},'%s','Delimiter','\n');            
        rows = rows{1};
        numLines = numel(rows);

        for idxLine = 1 : numLines 
            row = rows{idxLine};

            % replace top-level name with 'obj'
            %[~, rmn] = strtok(row,'.');
            %row = ['s' rmn];

            %row = ['s.' row];

            % deal with nonscalar nested structs/objs
            pat = '([\w]+)__([0123456789]+)\.';
            replc = '$1($2).';
            row = regexprep(row,pat,replc);

            % handle unencodeable value or nonscalar struct/obj
            unencodeval = '<unencodeable value>';
            if strfind(row,unencodeval)
                row = strrep(row,unencodeval,'[]');
            end

            % Handle nonscalar struct/object case
            nonscalarstructobjstr = '<nonscalar struct/object>';
            if strfind(row,nonscalarstructobjstr)
                row = strrep(row,nonscalarstructobjstr,'[]');
            end

            % TODO: Test this case further
            % handle ND array format produced by array2Str
            try 
                if ~isempty(strfind(row,'&'))
                    equalsIdx = strfind(row,'=');
                    [dimArr,rmn] = strtok(row(equalsIdx+1:end),'&');
                    arr = strtok(rmn,'&');
                    arr = reshape(str2num(arr),str2num(dimArr)); %#ok<NASGU,ST2NM>
                    eval([row(1:equalsIdx+1) 'arr;']);
                else
                    eval([row ';']);
                end
            catch ME %Warn if assignments to no-longer-extant properties are found
                if strcmpi(ME.identifier,'MATLAB:noPublicFieldForClass')
                    equalsIdx = strfind(row,'=');
                    warnMsg = sprintf(1,'Property ''%s'' was specified, but does not exist for class ''%s''\n', deblank(row(3:equalsIdx-1)),class(s));
                    most.idioms.warn(warnMsg);
                else
                    ME.rethrow();
                end
            end

            % Check if there is a value to assign
            if isempty(row)
                % This unassigned parameter value will be set to 0
                continue;
            end

            %% replace top-level name with 'obj'
            %[~, rmn] = strtok(row,'=');
            %evalStr = ['s.' frameStringCell{1} rmn]
            evalStr = ['s{' num2str(frameIdx) ',1}.' row];

            eval([evalStr ';']);
        end
    end
end


%--------------------------------------------------------------------------%
% parseHeaderStringCellToStruct.m                                          %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
