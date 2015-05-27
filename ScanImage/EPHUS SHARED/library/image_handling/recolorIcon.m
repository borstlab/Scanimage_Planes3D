% recolorIcon - Rescales colors in an icon, for use on a Matlab GUI.
%
% SYNTAX
%  cdata = recolorIcon(cdata, colorscaling)
%  cdata = recolorIcon(cdata, colorscaling, bgtriplet)
%   cdata - An NxNx3 uint8 array, the icon's data.
%   colorscaling - A vector of length 3, containing a corresponding multiplier for each color channel.
%   bgtriplet - The background color. Default: [212, 208, 200]
%
% USAGE
%
% NOTES
%  This is useful when making new icons, in order to quickly recolor them, without affecting the
%  shading or background color.
%  The background RGB triplet is assumed to be the default [212, 208, 200] (gray).
%
% CHANGES
%
% Created 10/6/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005

% %Example usage:
% path = 'C:\MATLAB704\work\library\gui_building\';
% color = {'red', 'green', 'blue'};
% colorscaling = [0.8, 0.5, 0.5; ...
%                 0.5, 0.8, 0.5; ...
%                 0.5, 0.5, 1];
% for direction = {'up', 'down', 'left', 'right'}
%     cdata = imread([path direction{1} '_arrow_embossed-16_16.bmp']);
%     for i = 1:3
%         cdata2 = recolorIcon(cdata, colorscaling(i, :), [212, 208, 200]);
%         imwrite(cdata2, [path direction{1} '_arrow_' color{i} '-16_16.bmp'], 'bmp');
%     end
% end
function cdata = recolorIcon(cdata, colorscaling, varargin)

if size(cdata, 3) ~= 3
    error('Invalid number of colors in cdata (must have 3 color channels).');
elseif length(colorscaling) ~= 3
    error('Invalid number of color scaling factors (must be 3).');
end

if isempty(varargin)
    bgcolor = [212, 208, 200];
else
    bgcolor = varargin{1};
    if length(bgcolor) ~= 3
        error('Invalid background color triplet (must be of length 3).');
    end
end

cdata = double(cdata);

background = intersect(intersect(find(cdata(:, :, 1) == bgcolor(1)), find(cdata(:, :, 2) == bgcolor(2))), find(cdata(:, :, 3) == bgcolor(3)));

for i = 1 : length(colorscaling)
    cdata(:, :, i) = cdata(:, :, i) * colorscaling(i);
end
cdata = round(cdata);

[y x] = ind2sub(size(cdata(:, :, 1)), background);
% figure, plot(x, y, '.')

%Why won't this work?!?
% cdata(x, y, 1) = bgcolor(1);
% cdata(x, y, 2) = bgcolor(2);
% cdata(x, y, 3) = bgcolor(3);
for i = 1 : 3
% bgcolor(i)
    for j = 1 : length(x)
% fprintf(1, '(%s, %s)\n', num2str(y(j)), num2str(x(j)));
        cdata(y(j), x(j), i) = bgcolor(i);
    end
end
% cdata(:, :, 1)
% cdata(:, :, 2)
% cdata(:, :, 3)

cdata = uint8(cdata);

return;