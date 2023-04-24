function th = pi2pi(varargin)

if nargin == 1
    th = deal(varargin{:});
    th = wrapToPi(th);
elseif nargin > 1
    [th,th_prev] = deal(varargin{:});
    if (th - th_prev)/th_prev < -2 && th < -pi/2 && th_prev > pi/2 && th_prev~=0
        th = th + 2*pi;
    elseif (th - th_prev)/th_prev > 2 && th > pi/2 && th_prev < -pi/2 && th_prev~=0
        th = th - 2*pi;
    end
end
