function [Aout,header] = genericOpenTif(filename,varargin)
% GENERICOPENTIF   - Opens an Image file with TIF extension.
%   GENERICOPENTIF opens a TIF file and store its contents as array Aout.
%   Filename is the file name with extension. 
%   
%   VARARGIN are paramameter value pairs that specify the output type.
%   Possible values include:
%
%    'filter'                1 or 0      Apply blocksize x blocksize Median Filter
%    'blocksize'             > 1         Blocksize for filter.
%    'splitIntoCellArray'    1 or 0      Output each input channel in cell array.
%    'linescan'              1 or 0      Reshape output into single frame by concatentating
%                                             to the bottom of the image.
%
%   See also CONVERTSTACKTOLS, PARSEHEADER

Aout=[];
header=[];

% Parse the inputs....
filter=0;
blocksize=3;
splitIntoCellArray=0;
linescan=0;
if nargin > 1
    % Parse input parameter pairs and rewrite values.
    counter=1;
    while counter+1 <= length(varargin)
        eval([varargin{counter} '=[(varargin{counter+1})];']);
        counter=counter+2;
    end
end

rgbImage = 0;

h = waitbar(0,'Opening Tif image...', 'Name', 'Open TIF Image', 'Pointer', 'watch');
try
    info=imfinfo(filename);
    frames = length(info);
    %TO081004b Tim O'Connor 8/10/04 - Files created in Photoshop (and presumably other programs) do not necessarily contain a header.
    if structFieldExists('info.ImageDescription')
        %TO051607A - Make this even more tolerant of non-ScanImage files. -- Tim O'Connor 5/16/07
        try
            header=info(1).ImageDescription;
            header=parseHeader(header);
        catch
            header = [];
            fprintf(2, 'genericOpenTif Warning - Failed to parse ScanImage header: %s.\n                         Header structure will be empty.\n', lasterr);
        end
    else
        fprintf(2, 'Warning: No header data found for tiff file: %s\n', filename);
    end

    for i = 1:frames
        waitbar(i/frames,h, ['Loading Frame Number ' num2str(i)]);    

        im = imread(filename, i);

        %TO081004c Tim O'Connor 8/10/04 - Look out for true-color tiff files (ie. from Photoshop).
        if size(im, 3) > 1
            rgbImage = 1;
            %This tolerance gives roughly 64k (65,536) colors (technically it should be 0.0254, but Matlab suggests using 0.025641).
            %The flooring rounds it off to 64,000 (exactly) with either number anyway. See `rgb2ind` for details.
            Aout(:, :, i) = rgb2ind(im, .0254, 'nodither');
%             im = mat2gray(double(im));
        else
            Aout(:,:,i) = im(:, :);
        end

    	if filter
			Aout(:,:,i)=medfilt2(Aout(:,:,i),[blocksize blocksize]);
		end
    end
    waitbar(1,h, 'Done');
    close(h);
catch
    close(h);
    %TO081004a Tim O'Connor 8/10/04 - Put in a useful error message.
    warning('Can not load file: %s\n Error: %s', filename, lasterr);
end

if rgbImage
    fprintf(2, 'Warning: Tiff file ''%s'' is in true-color RGB image format.\n An indexed grayscale/intensity image was expected but an RGB image was found.\n Converted image using uniform quantization method (see rgb2ind for details).\n Any color information has been discarded in order to create a 1-channel grayscale image.\n\n', filename);
end

% Pushes the data into cell arrays according to the number of channels read....
if splitIntoCellArray
    if isempty(header)
        %Discard color data.
        Aout = {Aout, [], []};
    else
        if isfield(header, 'acq')
            channels=header.acq.numberOfChannelsAcquire;
            for channelCounter=1:channels
                data{channelCounter}=Aout(:,:,channelCounter:channels:end);
            end
            Aout=data;
        else
            warning('Header for ''%s'' is missing the ''acq'' field. Splitting into cell array may yield unexpected results.', filename);
            Aout = {Aout, [], []};
        end
    end
end

if linescan
    if iscell(Aout)
        for j=1:length(Aout)
            Aout{j}=convertStackToLS(Aout{j});
        end
    else
        Aout=convertStackToLS(Aout);
    end
end

return;