function [y, dy, ddy,yta,ddyT] = transformationSystem(alphaY, betaY, goal, goalV,dt,dy, y,fi, tau,x,xEnd, alphaX,e)
% transformationSystem: compute a single step of the dynamical system.
% [y, dy, ddy] = transformationSystem(tauY, alphaY, betaY, goal, dy, y, tau)
% y'' = (1-x)*tauY*(alphaY*(betaY*(yta-y)+(goalV-y'));

yta = (goal + goalV*log(xEnd)/alphaX) -goalV.*log(x)/alphaX; %compute moving target
ddyT = ((1-x)^2)*tau^2*(alphaY*(betaY*(yta-y)+(goalV-dy))); % with moving target
ddy = ddyT+tau*fi+100*e; %adding f and coupling terms
%ddy = ((1-x)^2)*tau*(alphaY*(betaY*(goal-y)-dy))+tau*fi; % without moving target
y = (dt*dy*tau)+y;
dy = (dt*ddy*tau)+dy;

end