function c = C(n,th)
%% Functions with rotation matrices about each axis
if n==1
    c = [ones(1,1,size(th,2)) zeros(1,1,size(th,2)) zeros(1,1,size(th,2))
        zeros(1,1,size(th,2)) reshape(cos(th),[1,1,size(th,2)]) reshape(-sin(th),[1,1,size(th,2)])
        zeros(1,1,size(th,2)) reshape(sin(th),[1,1,size(th,2)]) reshape(cos(th),[1,1,size(th,2)])];
%     c = [1 0 0
%         0 cos(th) -sin(th)
%         0 sin(th) cos(th)];

elseif n==2
    c = [reshape(cos(th),[1,1,size(th,2)]) zeros(1,1,size(th,2)) reshape(sin(th),[1,1,size(th,2)])
        zeros(1,1,size(th,2)) ones(1,1,size(th,2)) zeros(1,1,size(th,2)) 
        reshape(-sin(th),[1,1,size(th,2)]) zeros(1,1,size(th,2)) reshape(cos(th),[1,1,size(th,2)])];
%     c = [cos(th) 0 sin(th)
%         0 1 0
%         -sin(th) 0 cos(th)];

elseif n==3
    c = [reshape(cos(th),[1,1,size(th,2)]) reshape(-sin(th),[1,1,size(th,2)]) zeros(1,1,size(th,2))
        reshape(sin(th),[1,1,size(th,2)]) reshape(cos(th),[1,1,size(th,2)]) zeros(1,1,size(th,2))
        zeros(1,1,size(th,2)) zeros(1,1,size(th,2)) ones(1,1,size(th,2))];
%     c = [cos(th) -sin(th) 0
%         sin(th) cos(th) 0
%         0 0 1];

else
    c = 'Error';
end

end