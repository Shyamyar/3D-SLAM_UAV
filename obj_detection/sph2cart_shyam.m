function cart = sph2cart_shyam(polar)
%SPH2CART Transform spherical to Cartesian coordinates.
%   CART = SPH2CART(POLAR) transforms corresponding elements of
%   data stored in spherical coordinates POLAR(azimuth TH, elevation PHI,
%   radius R) to Cartesian coordinates CART = [X,Y,Z].  The arrays TH, PHI, and
%   R must be the same size (or any of them can be scalar).  TH and
%   PHI must be in radians.
%
%   TH is the counterclockwise angle in the xy plane measured from the
%   positive x axis.  PHI is the elevation angle from the xy plane. 

[az,elev,r] = deal(polar(1,:),polar(2,:),polar(3,:));
z = r .* sin(elev);
rcoselev = r .* cos(elev);
x = rcoselev .* cos(az);
y = rcoselev .* sin(az);

cart = [x;y;z];