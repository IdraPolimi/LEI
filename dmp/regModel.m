function psi = regModel(x,c,rho, basisNumber,step)
if (step)
    psi = exp((-0.5*rho).*((x-c).^2));
else
    rhoM = repmat(rho,[size(x,2),1]);
    cM = repmat(c,[size(x,2),1]);
    xM = repmat(x,[basisNumber,1])';
    psi = exp((-0.5*rhoM).*((xM-cM).^2));
end
    