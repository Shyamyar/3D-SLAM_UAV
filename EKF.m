function [xhat,Phat]=EKF(xhat,Phat,Q,R,az,pz,qz,rz,phiz,thetaz,states,L_in,dt,g,prop_it,tn,sensor_noise)
%EKF
%Prediction------------------------------------------------------------
for j = 1:prop_it
    %xhat Prediction
    xhat=RKM_xhat_prediction(tn,xhat,az,pz,qz,rz,phiz,thetaz,g,dt/prop_it);
    %Phat prediction
    Phat=RKM_Phat_prediction(tn,xhat,az,phiz,thetaz,Q,Phat,dt/prop_it);
    tn=tn+dt/prop_it;
end

%Update----------------------------------------------------------------
for k=1:size(L_in,2)
    Ym(k,1)=Range_Sensor(states,L_in(:,k),sensor_noise);%Fetching measurements
    H=[(xhat(1)-L_in(1,k))/(sqrt((xhat(1)-L_in(1,k))^2+(xhat(2)-L_in(2,k))^2+(xhat(3)-L_in(3,k))^2)),...
        (xhat(2)-L_in(2,k))/(sqrt((xhat(1)-L_in(1,k))^2+(xhat(2)-L_in(2,k))^2+(xhat(3)-L_in(3,k))^2)),...
        (xhat(3)-L_in(3,k))/(sqrt((xhat(1)-L_in(1,k))^2+(xhat(2)-L_in(2,k))^2+(xhat(3)-L_in(3,k))^2)),0,0,0,0];
    K=Phat*H'*inv(R+(H*Phat*H'));
    Z=Ym(k,1)-sqrt((xhat(1)-L_in(1,k))^2+(xhat(2)-L_in(2,k))^2+(xhat(3)-L_in(3,k))^2)
    xhat=xhat+K*Z;
    Phat=(eye(size(xhat,1))-K*H)*Phat;
end
end