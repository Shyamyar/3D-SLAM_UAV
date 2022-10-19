function [xhat,Phat]=CDEKF(xhat,Phat,Q,R,az,pz,qz,rz,phiz,thetaz,states,dt,g,prop_it,tn,Ym,r)
%EKF
%Prediction------------------------------------------------------------
for j = 1:prop_it
    %xhat Prediction
    xhat(r)=RKM_xhat_prediction(tn,xhat(r),az,pz,qz,rz,phiz,thetaz,g,dt/prop_it);
    %Phat prediction
    Phat=RKM_Phat_prediction(tn,xhat(r),az,phiz,thetaz,Q,Phat,dt/prop_it);
    tn=tn+dt/prop_it;
end

%Update----------------------------------------------------------------
[phi,theta,psi] = deal(phiz,thetaz,-xhat(7));
C_BI = C(3,psi) * C(2,theta) * C(1,phi);    % 3-2-1 Rotation matrix for Body to Inertial
r_pos = [states([2,1]);-states(3)]; %[x,y,z] = [pe,pn,-pd]
L_in = C_BI * sph2cart_shyam(Ym) + r_pos; %[x,y,z]
for k=1:size(L_in,2)
    H=[(xhat(1)-L_in(2,k))/(sqrt((xhat(1)-L_in(2,k))^2+(xhat(2)-L_in(1,k))^2+(xhat(3)+L_in(3,k))^2)),...
        (xhat(2)-L_in(1,k))/(sqrt((xhat(1)-L_in(2,k))^2+(xhat(2)-L_in(1,k))^2+(xhat(3)+L_in(3,k))^2)),...
        (xhat(3)+L_in(3,k))/(sqrt((xhat(1)-L_in(2,k))^2+(xhat(2)-L_in(1,k))^2+(xhat(3)+L_in(3,k))^2)),0,0,0,0];
    K=Phat*H'*inv(R+(H*Phat*H'));
    Z=Ym(3,k)-sqrt((xhat(1)-L_in(2,k))^2+(xhat(2)-L_in(1,k))^2+(xhat(3)+L_in(3,k))^2);
    xhat(r)=xhat(r)+K*Z;
    Phat=(eye(size(xhat(r),1))-K*H)*Phat;
end
end