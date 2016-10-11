classdef TifStream < scanimage.interfaces.Class
    %% TIFSTREAM: Construct a tifstream object, which allows efficient 'streamed' writing to a TIF file (i.e. writing one frame at a time)
    %% SYNTAX
    %   ts = tifStream(filename, frameWidth, frameHeight)
    %   ts = tifStream(filename, frameWidth, frameHeight, imageDescription)
    %   ts = tifStream(filename, frameWidth, frameHeight, imageDescription, Property1, Value1, ...)
    %       filename: Valid TIF filename to which frames will be appended
    %       frameWidth,frameHeight: The width/heigh, in pixel count, of each frame of this TIF
    %       imageDescription: Optional string containing information about Tif file; this will be appended to each frame as its 'ImageDescription' tag. Omit or leave empty to skip this tag.
    %       Property1,Value1: Valid property/value pairs for tifstream objets
    %
    %  Settable Properties
    %       nominalStripSize: The number of samples per strip (default: 8K)
    %       dataType: One of uint8, int8, unit16, int16, uint32, int32 determining how frame data will be interpreted (default: int16)
    %       overwrite: true/false: determines if existing files with same file name are overwritten
    %
    %% NOTES
    %   This object serves to overcome the limitation of the Matlab imwrite() file for 'streaming' applications. When writing to a TIF file using imwrite(), even with the 'append' option,
    %   the file is opened anew for each call, and then the file must be scanned through before the new frame can be appended. Thus, the write time for each frame grows as the file does.
    %
    %   This object is designed specifically for the situation in ScanImage. The image size of each frame is identical and the same ImageDescription is repeated for each frame.
    %   Thus most of the tag data for each frame can be computed up front, outside of the appendFrame() call which occurs within the 'stream'.
    %
    %   All optional properties must be set on construction; there is no set() method for this class
    %
    %% CREDITS
    %   Created 8/16/08 by Vijay Iyer, 
    %   Changed to new style class on 9/28/10 by Jinyang Liu
    %% *************************************
    
    properties       
        imageDescription;
    end
    
    properties (SetAccess=private)
        fid;
        filename;
        
        imageDescriptionLength; %store this so that offset to 'stripOffsets' tag data can be computed
        imageDescriptionIFDByteDataIndices; % start and end index of the image description in the suppIFDByteData
        
        overwrite = false;
        nominalStripSize; %Determines target number of pixels to place in the strips comprising each frame
        dataType = 'int16';
        frameWidth;
        frameHeight;
        
        bytesPerImage;
        totalFrameCount = 0;
        currentFileFrameCount = 0;
        
        initialized = false;
        IFDByteData;
        suppIFDByteData;
        IFDOffsets;
        IFDOffsetVals;
        suppIFDOffsets;
        suppIFDOffsetVals;
        
        dataSigned;
        bitsPerSample;
        castFrameData;      % function handle to casting function; used to cast input frameData to specified data type
    end
    %%Constructor/destructor
    methods
        function obj = TifStream(filename, frameWidth, frameHeight, varargin)
            
            %Process input arguments
            obj.imageDescription = '';
            propNames = {};
            propVals = {};
            if ~isempty(varargin)
                if ~ischar(varargin{1})
                    error('Frame header must be specified as a string');
                else
                    assert(ischar('varargin{1}'),'Image description must be of type ''char''');
                    obj.imageDescription = varargin{1};
                    if length(obj.imageDescription) < 4 % pad to length of at least 4 characters
                        obj.imageDescription = [obj.imageDescription, repmat(char(10),1,4-length(obj.imageDescription))]; % pad with line feed ( char(10) )
                    end
                    
                    if mod(length(obj.imageDescription),2) %Ensure frameHeader is an even number of bytes and description is terminated with Null character
                        obj.imageDescription = [obj.imageDescription char(0)];
                    else
                        obj.imageDescription = [obj.imageDescription, char(10) , char(0)]; % pad with line feed ( char(10) )
                    end
                    varargin(1) = [];
                end
                if ~isempty(varargin)
                    if mod(length(varargin),2)
                        error('Invalid optional arguments. Must be property/value pairs.');
                    else
                        propNames = varargin(1:2:end);
                        propVals = varargin(2:2:end);
                        
                        if ~iscellstr(propNames)
                            error('Property names must be strings');
                        end
                    end
                end
            end
            
            if ~isempty(propNames)
                for i=1:length(propNames)
                    if ismember(propNames{i},{'nominalStripSize' 'dataType' 'overwrite'})
                        obj.(propNames{i}) = propVals{i}; %no error checking done on the value itself -- too much work!
                    else
                        most.idioms.dispError(['WARNING: Invalid property name (''' propNames{i} ''') ignored.\n']);
                    end
                end
            end
            
            %Initialize some class properties
            switch obj.dataType
                case 'uint8'
                    obj.dataSigned    = false;
                    obj.bitsPerSample = 8;
                    obj.castFrameData = @uint8;
                case 'int8'
                    obj.dataSigned    = true;
                    obj.bitsPerSample = 8;
                    obj.castFrameData = @int8;
                case 'uint16'
                    obj.dataSigned    = false;
                    obj.bitsPerSample = 16;
                    obj.castFrameData = @uint16;
                case 'int16'
                    obj.dataSigned    = true;
                    obj.bitsPerSample = 16;
                    obj.castFrameData = @int16;
                case 'uint32'
                    obj.dataSigned    = false;
                    obj.bitsPerSample = 32;
                    obj.castFrameData = @uint32;
                case 'int32'
                    obj.dataSigned    = true;
                    obj.bitsPerSample = 32;
                    obj.castFrameData = @int32;
                otherwise
                    assert('false','TifStream: Unsupported datatype: ''%s''',obj.dataType);
            end            
            
            obj.nominalStripSize = 8*1024; %Determines target number of pixels to place in the strips comprising each frame
            obj.frameWidth = frameWidth;
            obj.frameHeight = frameHeight;
            obj.imageDescriptionLength = length(obj.imageDescription); %store this so that offset to 'stripOffsets' tag data can be computed
            assert(obj.imageDescriptionLength <= 2^32-1); % Sanity check
            obj.bytesPerImage = obj.frameWidth*obj.frameHeight*(obj.bitsPerSample/8);
            
            
            obj.filename = filename;
            %Create TIF file
            try
                obj.createTifFile();
            catch
                obj.fid = [];
                obj.filename = '';
                error('Unable to construct %s instance: %s',mfilename('class'),lasterr);
            end
            
            obj.computeIFDByteData();
            
            obj.initialized = true;
        end
        
        
        function delete(obj)                        
            %Close file, if it exists and is open
            try
                if ~isempty(obj.fid)
                    fclose(obj.fid);
                    delete(obj.filename);
                end
            catch
                error(['Error in closing or deleting file, so the file may remain: ' lasterr]);
            end
        end
        
    end
    
    methods
        function value = get(obj, varargin)
            
            names = fieldnames(obj);
            
            if isempty(varargin)
                for i = 1 : length(names)
                    value.(names{i}) = obj.(names{i});
                end
            elseif ~iscellstr(varargin)
                error('Arguments must all be strings, specifying %s property names',mfilename('class'));
            else
                for i=1:length(varargin)
                    [found, idx] = ismember(lower(varargin{i}), lower(names));
                    if ~found
                        error('Invalid property name: ''%s''', varargin{i});
                    else
                        value{i} = obj.(names{idx});
                    end
                end
            end
            
            %Don't return cell array if only one property requested
            if iscell(value) && length(value)==1 %VI042809A
                value = value{1};
            end
        end
        
        function oldfilename = newFile(obj,fileName)
            if strcmp(obj.filename,fileName)
                return
            end
            
            oldfilename = obj.closeTifFile();
            obj.filename = fileName;
            %Create TIF file
            try
                obj.createTifFile();
            catch
                obj.fid = [];
                obj.filename = '';
                error('Unable to construct %s instance: %s',mfilename('class'),lasterr);
            end
            
            obj.currentFileFrameCount = 0;
        end
        
        function appendFrame(obj,frameData,frameDataTransposed)
            %  Append frame of data to TIF file stream
            %% SYNTAX
            %   hTifStream.appendFrame(frameData)
            %       hTifStream: A handel of the tifStream object
            %       frameData: A matrix of image data, comprising one 'frame'. Must be of the size specified by the @scim_tifStream's imageHeight and imageWidth proprties.
            %
            %% NOTES
            %   Data is converted to the type specified by the @scim_tifStream's pixelDataType property
            if nargin < 3 || isempty(frameDataTransposed)
                frameDataTransposed = false;
            end
            
            frameOffset = ftell(obj.fid);
            
            %% Append actual frame data
            if ~frameDataTransposed % reversed logic
                frameData = frameData';
            end
            
            %% Checks
            if size(frameData,2) ~= obj.frameHeight || size(frameData,1) ~= obj.frameWidth
                obj.close();
                error('Supplied frame is of incorrect size. File stream has been closed & deleted.');
            end
            
            %% Data type conversion
            frameData = obj.castFrameData(frameData);
            
            fwrite(obj.fid, frameData,class(frameData));
            
            %% Adjust IFD & Supplemental IFD data
            
            for i=1:length(obj.IFDOffsets)
                val = obj.IFDOffsetVals(i) + frameOffset;
                indices = obj.IFDOffsets(i):(obj.IFDOffsets(i)+3);
                
                obj.IFDByteData(indices) = obj.makeByteArray(val,4);
            end
            
            for i=1:length(obj.suppIFDOffsets)
                val = obj.suppIFDOffsetVals(i) + frameOffset;
                indices = obj.suppIFDOffsets(i):(obj.suppIFDOffsets(i)+3);
                
                obj.suppIFDByteData(indices) = obj.makeByteArray(val,4);
            end
            
            
            %% Append IFD & Supplementary IFD Data
            fwrite(obj.fid, obj.IFDByteData, 'uint8');
            fwrite(obj.fid, obj.suppIFDByteData, 'uint8');
            
            obj.totalFrameCount = obj.totalFrameCount + 1;
            obj.currentFileFrameCount = obj.currentFileFrameCount + 1;
        end
        
        function filename = close(obj)
            filename = obj.closeTifFile();
            
            if obj.currentFileFrameCount == 0 && ~isempty(obj.fid)
                try
                    delete(obj.filename);
                    filename = [];
                catch ME
                    most.idioms.reporError(ME);
                end
            end
            
            obj.delete(); %This removes the tifstream object
        end
        
    end
    

    %% set/get methods
    methods
        function set.imageDescription(obj,val)
            if ~obj.initialized
                obj.imageDescription = val;
                return
            end
            
            % make sure the image description was initialized in the constructor
            assert(obj.imageDescriptionLength > 0,'Cannot overwrite empty image description');
            
            % truncate/pad the new image description to the right right size
            inputLength = length(val);
            if inputLength < obj.imageDescriptionLength
                % pad description with Nul characters
                paddingBytes = repmat(' ',1,obj.imageDescriptionLength - inputLength);
                val = [val paddingBytes];
            elseif inputLength > obj.imageDescriptionLength
                val = val(1:obj.imageDescriptionLength);
                most.idioms.dispError('Warning: Image description was truncated to the preallocated size\n');
            end
            
            val(end) = char(uint8(0)); % image description string must be terminated with NULL character
            
            obj.imageDescription = val;
            
            % replace the image description in suppIFDByteData
            startIndex = obj.imageDescriptionIFDByteDataIndices(1);
            endIndex = obj.imageDescriptionIFDByteDataIndices(2);
            obj.suppIFDByteData(startIndex:endIndex) = obj.imageDescription;            
        end
    end
    
    methods (Access = private)
        
        function createTifFile(obj)
            
            [path,fname,ext] = fileparts(obj.filename);
            nonTifExt = false;
            if isempty(ext)
                obj.filename = [obj.filename '.tif'];
            elseif ~strcmpi(ext,'.tif')
                nonTifExt = true;
            end
            
            if ~obj.overwrite && exist(obj.filename,'file')
                error(['File ' obj.filename ' already exists. A ''scim_tifStream'' object can only be created for a new file.']);
            end
            
            obj.fid = fopen(obj.filename,'W');
            if obj.fid < 0
                error(['Failed to create file ' obj.filename '. Could not construct ' mfilename ' object.']);
            end
            if nonTifExt
                most.idioms.dispError(['File '  fname ext ' does not have a TIF extension, but is a TIF file nonethless.']);
            end
            
            %Write TIF header data
            fwrite(obj.fid,uint8('II')); %Specifies that little-endian byte-ordering is used
            fwrite(obj.fid,42,'uint16'); %Specifies TIFF format version
                        
            fwrite(obj.fid,8+obj.bytesPerImage,'uint32'); %Write offset to first IFD
        end
        
        function filename = closeTifFile(obj)
            % Complete and close the file associated with this @tifstream
            
            lengthSuppIFD = length(obj.suppIFDByteData);
            
            %Adjust the last offset to 'next' IFD to 0000
            fseek(obj.fid,-(lengthSuppIFD+4),'eof');
            fwrite(obj.fid,0,'uint32');
            fclose(obj.fid);
            
            obj.fid = [];
            filename = obj.filename;
        end
        
        function computeIFDByteData(obj)
            
            %Fixed Tag values
            compression = 1; %no compression
            photometricInterpretation = 1; %blackIsZero
            orientation = 1; %topLeft
            samplesPerPixel = 1;
            xResolution = [72*16^6 1*16^6]; %resolution = 72
            yResolution = [72*16^6 1*16^6]; %resolution = 72
            planarConfig = 1; %'chunky'
            resolutionUnit = 2; %inches
            
            %Computed tag values
            imageWidth = obj.frameWidth;
            imageLength = obj.frameHeight;
            
            if obj.dataSigned
                sampleFormat = 2;
            else
                sampleFormat = 1;
            end
            
            rowsPerStrip = round(obj.nominalStripSize/imageWidth);
            
            numStrips = max(floor(imageLength/rowsPerStrip),1);
            
            stripByteCounts = repmat(imageWidth*min(rowsPerStrip,imageLength)*obj.bitsPerSample/8,1,numStrips);
            finalStripRows = mod(imageLength,rowsPerStrip);
            if finalStripRows
                numStrips = numStrips+1;
                stripByteCounts = [stripByteCounts imageWidth*finalStripRows*obj.bitsPerSample/8];
            end
            
            %Initialize IFD Byte Data
            obj.IFDByteData = uint8([]);
            if ~isempty(obj.imageDescription)
                numTags = 16;
            else
                numTags = 15;
            end
            obj.IFDByteData = [obj.IFDByteData obj.makeByteArray(numTags,2)];
            
            %Append tags in sequence
            obj.suppIFDByteData = uint8([]);
            obj.IFDOffsets = [];
            obj.IFDOffsetVals = [];
            
            appendValueTag(256,3,1,imageWidth);
            appendValueTag(257,3,1,imageLength);
            appendValueTag(258,3,1,obj.bitsPerSample);
            appendValueTag(259,3,1,compression);
            appendValueTag(262,3,1,photometricInterpretation);
            if ~isempty(obj.imageDescription)
                obj.imageDescriptionIFDByteDataIndices = appendOffsetTag(270,2,length(obj.imageDescription),obj.imageDescription);
            end
            
            if length(stripByteCounts) > 1
                appendOffsetTag(273,4,length(stripByteCounts),zeros(1,length(stripByteCounts))); %stripOffsets tag -- use placeholder value (0) here. Actual value filled in during appendFrame().
            else
                tagOffset = appendValueTag(273,4,1,0); %stripOffsets tag -- use placeholder value (0) here. Actual value filled in during appendFrame().
                obj.IFDOffsets = [obj.IFDOffsets,tagOffset];
                obj.IFDOffsetVals = [obj.IFDOffsetVals, 0];
            end
            appendValueTag(274,3,1,orientation);
            appendValueTag(277,3,1,samplesPerPixel);
            appendValueTag(278,3,1,rowsPerStrip);
            if length(stripByteCounts) > 1
                appendOffsetTag(279,4,length(stripByteCounts),stripByteCounts);
            else
                appendValueTag(279,4,1,stripByteCounts);
            end
            appendOffsetTag(282,5,1,xResolution);
            appendOffsetTag(283,5,1,yResolution);
            appendValueTag(284,3,1,planarConfig);
            appendValueTag(296,3,1,resolutionUnit);
            appendValueTag(339,3,samplesPerPixel,sampleFormat);

            %Add offset to next IFD at end of IFD
            obj.IFDOffsets = [obj.IFDOffsets length(obj.IFDByteData)+1];
            obj.IFDByteData = [obj.IFDByteData obj.makeByteArray(0,4)]; %place-holder vale. Actual value filled in during appendFrame()
            obj.IFDOffsetVals = [obj.IFDOffsetVals 2*obj.bytesPerImage+length(obj.IFDByteData)+length(obj.suppIFDByteData)]; %value should be augmented by frameOffset for each frame
            
            %Determine supplementary IFD offset locations and values -- these values should be augmented by frameOffset for each frame
            obj.suppIFDOffsets = [1:4:(4*(length(stripByteCounts)-1)+1)] + length(obj.imageDescription);
            obj.suppIFDOffsetVals = cumsum(stripByteCounts)-stripByteCounts(1);
            
            %Append tag data for cases where data fits into 4 bytes
            function byteDataStartIndex = appendValueTag(tagID,type,count,data)
                tagID = obj.makeByteArray(tagID, 2);
                type = obj.makeByteArray(type,2);
                count = obj.makeByteArray(count,4);
                data = obj.makeByteArray(data,4);
                
                obj.IFDByteData = [obj.IFDByteData tagID type count data];
                
                byteDataStartIndex = length(obj.suppIFDByteData) - 4 + 1;
            end
            
            %Append tag data for cases where data runs beyond 4 bytes
            function suppIFDByteDataIndices = appendOffsetTag(tagID,type,count,data)
                
                switch type
                    case 2
                        tagData = uint8(data);
                        numBytes = 1;
                    case 3
                        tagData = uint16(data);
                        numBytes = 2;
                    case 4
                        tagData = uint32(data);
                        numBytes =4;
                    case 5
                        if mod(length(data),2)
                            error('Rational tag data must be supplied as pairs of values');
                        end
                        tagData = uint32([]);
                        for i=1:length(data)/2
                            tagData = [tagData uint32(data(2*i-1)) uint32(data(2*i))];
                            data(2*i-1:2*i) = [];
                        end
                        numBytes = 4;
                    otherwise
                        error('Invalid TIF tag data type');
                end
                
                %Make Byte Data
                tagID = obj.makeByteArray(tagID, 2);
                type = obj.makeByteArray(type,2);
                count = obj.makeByteArray(count,4);
                obj.IFDByteData = [obj.IFDByteData tagID type count];
                
                obj.IFDOffsets = [obj.IFDOffsets length(obj.IFDByteData)+1]; %Relative offset of offset to adjust
                obj.IFDOffsetVals = [obj.IFDOffsetVals length(obj.suppIFDByteData) + numTags*12 + 6 + obj.bytesPerImage];
                
                offset = obj.makeByteArray(0,4); %Placeholder value -- actual value will be computed during appendFrame()
                obj.IFDByteData = [obj.IFDByteData offset];
                
                byteDataStartIndex = length(obj.suppIFDByteData) + 1;
                obj.suppIFDByteData = [obj.suppIFDByteData obj.makeByteArray(tagData,numBytes)];
                byteDataEndIndex = length(obj.suppIFDByteData);
                
                suppIFDByteDataIndices = [byteDataStartIndex byteDataEndIndex];
            end
            
        end
        
        function byteArray = makeByteArray(obj,val,numBytes)
            % Make an array of bytes of specified length from the specified value
            %% SYNTAX
            %   byteArray = makeByteArray(val,numBytes)
            %       val: Value, or array of values, to be written out
            %       numBytes: Number of bytes to write per value; zeros will be appended if needed
            %       byteArray: Array of bytes (i.e. 'uint8' type) representing value, with zeros appended as required
            %
            %% *************************************************

            
            byteArray = zeros(numBytes,length(val),'uint8');
            for i=1:length(val)
                %Limits checking
                if val(i) > 2^(numBytes*8)-1
                    error(['Value (' val(i) ') exceeds that allowed for specified number of bytes']);
                end
                %val(i) = uint32(i);
                val(i) = double(val(i));

                byteVals = zeros(numBytes,1,'uint8');
                byteCount = 0;
                for j=numBytes:-1:1
                    byteVals(j) = uint8(floor(val(i)/(2^(8*(j-1)))));
                    val(i) = val(i) - double(byteVals(j))*2^(8*(j-1));
                    if byteVals(j) ~= 0
                        byteCount = max(byteCount,j);
                    end
                end
                
                %byteArray = [byteArray byteVals]; %append to array
                byteArray(:,i) = byteVals;
            end
            byteArray = reshape(byteArray,1,[]);
        end
    end
end



%--------------------------------------------------------------------------%
% TifStream.m                                                              %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
