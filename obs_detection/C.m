function c = C(n,th)
%% Functions with rotation matrices about each axis
if n==1
    c = [1 0 0
        0 cos(th) -sin(th)
        0 sin(th) cos(th)];

elseif n==2

    c = [cos(th) 0 sin(th)
        0 1 0
        -sin(th) 0 cos(th)];

elseif n==3

    c = [cos(th) -sin(th) 0
        sin(th) cos(th) 0
        0 0 1];

else
    c = 'Error';
end

end