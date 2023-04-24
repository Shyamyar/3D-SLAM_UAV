function [xx,yy,zz] = ell_cone_3D(varargin)
% CONE Generate 3D Cone ELLIPSOID.
%   [X,Y,Z] = CONE_ELL_3D(R,H,N) forms the unit elliptical 3D cone based 
%   on the generator curve in the vector R.
%   Vector R contains the radius at equally spaced points along the H 
%   height of the cone.
%   The cone has N points around the circumference. SURF(X,Y,Z) displays 
%   the cone.
%
%   [X,Y,Z] = FOV_3D(R), and [X,Y,Z] = FOV_3D default to N = 20
%   and R = [0 1].
%
%   Omitting output arguments causes the cone to be displayed with
%   a SURF command and no outputs to be returned.
%
%   ELL_CONE_3D(AX,...) plots into AX instead of GCA.

% Parse possible Axes input
narginchk(0,4);
[cax,args,nargs] = axescheck(varargin{:});

n = 100;
a = [0 1 4 9 16]';
b = [0 2 8 18 32]';
h = 1;
if nargs > 0, a = args{1}; end
if nargs > 1, b = args{2}; end
if nargs > 2, h = args{3}; end
if nargs > 3, n = args{4}; end
a = a(:); % Make sure a is a vector.
b = b(:); % Make sure b is a vector.
m_a = length(a); m_b = length(b);
if m_a~=m_b
    sprintf('Error, size mismatch between a and b');
end
if m_a==1, a = [0;a]; b = [0,b]; m_a = 2; end
theta = (0:n)/n*2*pi;
sintheta = sin(theta); sintheta(n+1) = 0;

Rot = C(1,deg2rad(0)); % For rotation of ellipse about X-axis
y = a * cos(theta);
z = b * sintheta;
x = (0:m_a-1)'/(m_a-1) * ones(1,n+1) * h;

for i = 1:m_a
    points = Rot * [x(i,:);y(i,:);z(i,:)];
    x(i,:) = points(1,:);
    y(i,:) = points(2,:);
    z(i,:) = points(3,:);
end

if nargout == 0
    cax = newplot(cax);
    surf(x,y,z,'parent',cax)
else
    xx = x; yy = y; zz = z;
end