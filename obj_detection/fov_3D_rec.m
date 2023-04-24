function [xx,yy,zz] = fov_3D_rec(varargin)
% Generate rec FOV using range limit (h) along X-axis, and exscribe
% rectangle outside ellipse with angular limits on azimuth and elevation.
%
%   Defaults are h=1 and [-45,45] deg on azimuth and [-30,30] deg on 
%   elevation.
%   
%   Omitting output arguments causes the cone to be displayed with
%   a SURF command and no outputs to be returned.
%
%   FOV_3D_REC(AX,...) plots into AX instead of GCA.

% Parse possible Axes input
narginchk(0,3);
[cax,args,nargs] = axescheck(varargin{:});

n = 4;
h = 1;
az = deg2rad(45);
elev = deg2rad(30);
if nargs > 0, az = args{1}; end
if nargs > 1, elev = args{2}; end
if nargs > 2, h = args{3}; end
a = [0 h*tan(az)]';
b = [0 h*tan(elev)]';
a = a(:); % Make sure a is a vector.
b = b(:); % Make sure b is a vector.
m = 2;    % Two plane, one at vertex and 
theta = (0:n)/n*2*pi;
sintheta = sin(theta); sintheta(n+1) = 0;

Rot = C(1,deg2rad(0)); % For rotating the prism to FOV setup
yi = a * cos(theta);
zi = b * sintheta;
x = (0:m-1)'/(m-1) * ones(1,n+1) * h;
y = [yi(:,1) yi(:,3) yi(:,3) yi(:,1) yi(:,1)];
z = [zi(:,2) zi(:,2) zi(:,4) zi(:,4) zi(:,2)];

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
