function createConfigFile(bitFlags, fid, outputFlag)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% NOTES
%   1-D arrays of data are now stored as strings in variables ending in 'ArrayString'. Their 
%   underlying variable has the same name, but ending in 'Array'. 
%
%%  CHANGES 
%      12/15/03 by Tim O'Connor - Bug fix (see below).
%       TPMOD_1: Modified 12/31/03 Tom Pologruto - Handles defFile Input correctly now       
%       VI012909A: Implement Array/ArrayString convention for 1D arrays -- Vijay Iyer 1/29/09
%       VI013009A: Save N-d arrays as a 'string string' so they can be properly parsed by initGUIsFromCellArray() -- Vijay Iyer 1/30/09
%       VI013009B: Only save N-d arrays when greater than 2D; use mat2str() instead of num2str() for <=2D arrays -- Vijay Iyer 1/30/09
%       VI013009C: Enclose all arrays in quotes while scalar/empty values can be left un-quoted; this is to deal correctly with behavior of tokenize() in initGUIsFromCellArray() -- Vijay Iyer 1/30/09
%       VI031609A: Utilize the refactored stateVar2String() function -- Vijay Iyer 3/16/09
%
%% CREDITS
%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%% *************************************************************

global configGlobals
if isstruct(configGlobals)
    fNames=fieldnames(configGlobals);
    for i=1:length(fNames)
        recurseCreateConfigFile(fNames{i}, bitFlags, '', fid, outputFlag);
    end
end
recurseCreateConfigFile('state', bitFlags, '', fid, outputFlag);


function recurseCreateConfigFile(startingName, bitFlags, pad, fid, outputFlag)
if length(startingName)==0
    return
end

[topName, structName, fieldName]=structNameParts(startingName);
eval(['global ' topName]);

%%%VI100110A: No ineligible cell array vars should be flagged for saving anyway 
% if eval(['iscell(' startingName ');'])
%     return
% end

if length(fieldName)==0
    fieldName=topName;
end

if eval(['~isstruct(' startingName ');']) || ~isempty(strfind(startingName,'Struct')) %VI012411A
    if any(bitand(getGlobalConfigStatus(startingName),bitFlags)) | bitFlags==0			% if 0, output everything for ini file
        val = stateVar2String(startingName); %VI031609A
        %%%VI031609A: Removed %%%%%%%%%%%%%
        %         val=[];
        %         if strfind(startingName,'ArrayString')
        %             eval(['val= mat2str(' startingName(1:end-6) ');']);
        %         else
        %             eval(['val=' startingName ';']);
        %         end
        %         if isnumeric(val)
        %             %if length(val)>1
        %             if ndims(val) > 2 %VI013009B
        %                 %val=['[' num2str(val) ']'];
        %                 % This statement was changed because it generated an error when
        %                 % trying to process the state.init.eom.powerTransitions.protocols 4D array.
        %                 % The error message was: Error using ==> horzcat
        %                 %                        All matrices on a row in the bracketed expression must have the same number of rows.
        %                 %
        %                 % Ideally, that array does not need saving, since the same information is packed into a string.
        %                 %
        %                 % Tim O'Connor - 12/15/03
        %                 % val = strcat('[', num2str(val), ']');
        %                 %Changed again - Tim O'Connor 3/30/04 TO33004a
        %                 val = ['''' ndArray2Str(val) '''']; %VI013009A
        %             elseif isscalar(val) || isempty(val) %VI013009C
        %                 val=mat2str(val); %VI013009B
        %             else
        %                 val = ['''' mat2str(val) '''']; %VI013009B, VI013009C
        %             end
        %         else
        %             val=['''' val ''''];
        %         end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        if outputFlag==0
            fprintf(fid, '%s\n', [pad fieldName]);
        else
            fprintf(fid, '%s=%s\n', [pad fieldName], val);
        end				
    end
else
    if ~exist(topName, 'var')
        return 
    end
    if length(fieldName)==0
        fieldName=topName;
    end
    fprintf(fid, [pad 'structure ' fieldName '\n']);
    fNames=[];
    eval(['fNames=fieldnames(' startingName ');']);
    for i=1:length(fNames)
        if ~any(strcmp(fNames{i}, {'configGlobals', 'globalGUIPairs'}))
            recurseCreateConfigFile([startingName '.' fNames{i}], bitFlags, [pad '   '], fid, outputFlag);
        end
    end
    fprintf(fid, [pad 'endstructure\n']);
end
