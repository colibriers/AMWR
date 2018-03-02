function [ cartxy ] = Polar2Cart( range,dataScope,resol)
%Polar2Cart Summary of this function goes here
% Transfor the polar rho/theta to cartesian x/y in the dataScope
transforNum = (dataScope(2) - dataScope(1))/resol + 1;
rad = (dataScope(1):resol:dataScope(2))*pi/180;
cartxy = zeros(2,transforNum);
for i=1:1:transforNum
    cartxy(:,i) = range(1,i)*[cos(rad(i));sin(rad(i))];
end

end

