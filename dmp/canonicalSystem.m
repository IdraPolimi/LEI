function [x] = canonicalSystem(x, dt, alphax,tau)
% canonicalSystem: compute the phase variable x for each time instant;
% [x] = canonicalSystem(x, dt, alpha, tau)
% x' = tau* -alphax *x

x = tau*dt*(-alphax*x)+x;

end