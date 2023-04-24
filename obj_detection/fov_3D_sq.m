function [xx,yy,zz] = fov_3D_sq(varargin)
% Generate square FOV using range limit (h) along X-axis, and inscribe
% square in circle of radius r.
%   
%   Defaults are h = 1, r = [0 1]' and n = 4. FOV Angular limits are
%   [-45,45] deg.
%   Omitting output arguments causes the cone to be displayed with
%   a SURF command and no outputs to be returned.
%
%   FOV_3D_SQ(AX,...) plots into AX instead of GCA.

% Parse possible Axes input
narginchk(0,2);
[cax,args,nargs] = axescheck(varargin{:});

n = 4;
r = [0 1]';
h = 1;
if nargs > 0, r = args{1}; end
if nargs > 1, h = args{2}; end
r = r(:); % Make sure r is a vector.
m = length(r); if m==1, r = [0;r]; m = 2; end
theta = (0:n)/n*2*pi;
sintheta = sin(theta);
sintheta(n+1) = 0;

Rot = C(1,deg2rad(45));
y = r * cos(theta);
z = r * sintheta;
x = (0:m-1)'/(m-1) * ones(1,n+1) * h;

pointsA = Rot * [x(1,:);y(1,:);z(1,:)];
pointsB = Rot * [x(2,:);y(2,:);z(2,:)];
x = [pointsA(1,:);pointsB(1,:)];
y = [pointsA(2,:);pointsB(2,:)];
z = [pointsA(3,:);pointsB(3,:)];

if nargout == 0
    cax = newplot(cax);
    surf(x,y,z,'parent',cax)
else
    xx = x; yy = y; zz = z;
end
