function fillscreen(hFig)
% FILLSCREEN   - Sets a figure size to completely fill the screen.
%
%   FILLSCREEN(hFig) sets figure with handle hFig to completely fill the screen
%
% See also SHOWMETHEFIGS, SPLAYFIGS, FIGSHIFT

if nargin==0
    hFig = gcf;
end;

res=get(0,'ScreenSize');
set(hFig,'Position',[1 1 res(3) res(4)-64]); %Leave room for title bar
