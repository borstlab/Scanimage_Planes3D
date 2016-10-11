function splayAxisVertical(f,varargin)
% SPLAYAXISVERTICAL   - Displays axes in a column (like subplot(M,1)).
%   SPLAYAXISVERTICAL displays all the axes that are children of the 
%   figure handle f in a vertical row with parameters specifed.
%
%   varargin are parameter (string) value (numeric) pairs that specify:
%
%       gap: fractional space between axes (0 to 1)
%
% 	    start: fractional space between bottom edge of figure and bottom of
% 	    1st axes.
%
% 	    height: fractional space occupied by all axes.
%
%	See also SPLAYAXISTILE, SPLAYAXISHORIZONTAL, SPLAYFIGS, SUBPLOT

if nargin == 0
    f=gcf;  
else
    if mod(length(varargin),2)==1   % passed a f...
        varargin=[{f} varargin];
        f=gcf;   
   end    
end

allAx=findobj(f, 'Type', 'axes', 'Box', 'off');
nAx=length(allAx);
if nAx==0
	return
end

gap=.1;
start=.1;
height=0.81;

% Parse input parameter pairs and rewrite values.
counter=1;
while counter+1 <= length(varargin)
    eval([varargin{counter} '=' num2str(varargin{counter+1}) ';']);
    counter=counter+2;
end



delta=(height-gap*(nAx-1))/nAx;

for counter=1:length(allAx)
	set(allAx(nAx-counter+1), 'Position', [.1 start+(counter-1)*(delta+gap) .8 delta]);
end