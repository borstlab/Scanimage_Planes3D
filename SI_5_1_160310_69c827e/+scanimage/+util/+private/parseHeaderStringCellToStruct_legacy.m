function [s, numHdrs, headerLeadStr] = parseHeaderStringCellToStruct_legacy(frameStringCell,verInfo)
%
    s = struct();
    numHdrs = 0;
    numImg = 0;

    [headerLeadStr] = scanimage.util.private.getPerFrameHeaderPropsFromVersion(verInfo);
    numImg = numel(frameStringCell);
    numHdrs = numel(headerLeadStr);

    % NOTE: We will soon no longer assume any structure in the header. We will just return a cell
    %       array containing each of the properties found in the header. This is for the sake of 
    %       stability.

    for idxHdr = 1:numel(headerLeadStr.forArray)
        % Unassigned values are set to zero
        eval(['s.' headerLeadStr.forArray{idxHdr} ' = zeros(1,numImg);'])
    end

    for idxHdr = 1:numel(headerLeadStr.forCellArray)
        % Unassigned values are set to zero
        eval(['s.' headerLeadStr.forCellArray{idxHdr} ' = cell(1,numImg);'])
    end

    for frameIdx = 1:1
        rows = textscan(frameStringCell{frameIdx},'%s','Delimiter','\n');            
        rows = rows{1};
        % Process the rest of the header properties
        for idxLine = numel(headerLeadStr.forArray) + numel(headerLeadStr.forCellArray) + 1 : numel(rows)
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
            evalStr = ['s.' row];

            eval([evalStr ';']);
        end
    end

    for frameIdx = 1:numImg
        rows = textscan(frameStringCell{frameIdx},'%s','Delimiter','\n');            
        rows = rows{1};

        % Process header properties whose names need to be changed
        for idxLine = 1 : numel(headerLeadStr.forArray)
            row = rows{idxLine};

            %% replace top-level name with 'obj'
            [~, rmn] = strtok(row,'=');

            % Check if there is a value to assign
            if strcmp(strtrim(rmn),'=') ;
                % This unassigned parameter value will be set to 0
                continue;
            end

            evalStr = ['s.' headerLeadStr.forArray{idxLine} '(' num2str(frameIdx) ')'  rmn];

            eval([evalStr ';']);
        end

        for idxCellArray = 1 : numel(headerLeadStr.forCellArray)
            idxLine = idxCellArray + numel(headerLeadStr.forArray);
            row = rows{idxLine};

            %% replace top-level name with 'obj'
            [~, rmn] = strtok(row,'=');

            % Check if there is a value to assign
            if strcmp(strtrim(rmn),'=') ;
                % This unassigned parameter value will be set to 0
                continue;
            end

            evalStr = ['s.' headerLeadStr.forCellArray{idxCellArray} '{' num2str(frameIdx) '}'  rmn];

            eval([evalStr ';']);
        end
    end

end


%--------------------------------------------------------------------------%
% parseHeaderStringCellToStruct_legacy.m                                   %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
