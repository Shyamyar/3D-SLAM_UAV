function [polar, beta_xy] = cart2sph_shyam(cart)
%CART2SPH Transform Cartesian to spherical coordinates.
%   [POLAR, ELEV_XY] = CART2SPH_SHYAM(CART) transforms corresponding elements of
%   data stored in Cartesian coordinates CART = [X, Y, Z] to spherical
%   coordinates POLAR = [TH, PHI, R](azimuth TH, elevation PHI, and radius
%   R). The arrays X, Y, and Z must be the same size (or any of them can
%   be scalar). TH and PHI are returned in radians.
%   
%   There is a another output argument called ELEV_XY that gives the angle
%   of landmark from XY plane considering a plane passing through the
%   landmark and Y-axis. This is for comparing to FOV in Elevation.
%
%   TH is the counterclockwise angle in the xy plane measured from the
%   positive x axis.  PHI is the elevation angle from the xy plane.

[x(1, :), y(1, :), z(1, :)] = deal(cart(1, :), cart(2, :), cart(3, :));
hypotxy = hypot(x, y);
alpha = atan2(y, x);
beta = atan2(z, hypotxy);
beta_xy = atan2(z, x);
delta = hypot(hypotxy, z);

polar = [alpha; beta; delta];
