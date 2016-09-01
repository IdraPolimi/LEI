function [psi] = regModel(x,rho,c)
% regModel: compute the regression model exp(0.5*rho*(x-c)^2)
%rho = (1./rho).^2

rhoM = repmat(rho,[size(x,2),1]);
cM = repmat(c,[size(x,2),1]);
xM = repmat(x,[15,1])';

psi = exp((-0.5*rhoM).*((xM-cM).^2));
end