function [c,rho] = regModelParam(x,T,tPercentage,basisNumber,alphaX)

disp('Generating gaussians params...');

%% computing centers of gaussians
cTemp = linspace(1,T*tPercentage,basisNumber+1); %generate a basisNumber of centers equally spaced over O:T
cTemp(end)=[];%delete last center
c = x(round(cTemp));

%% computing variable variance of gaussians
rho = (11/alphaX)*basisNumber./(c.^2); %variable variance for compensating exponential "time"


% %% computing gaussians
% rhoM = repmat(rho,[size(x,2),1]);
% cM = repmat(c,[size(x,2),1]);
% xM = repmat(x,[basisNumber,1])';
% psi = exp((-0.5*rhoM).*((xM-cM).^2));

disp('Gaussians params generated!');
clear ii
end