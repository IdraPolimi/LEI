function [w] = learnTrajectory(ytg, dytg, ddytg, alphaY, betaY,psi, x,alphaX,figNum)
% learnTrajectory: given a trajectory in terms of position, velocity and acceleration,
% learns the weighted PSI

goal = ytg(end);
goalV = dytg(end);
%init = ytg(1);
%initV = dytg(1);

%% Computing target function from target trajectory
yta = (goal + goalV*log(x(end))/alphaX) -goalV.*log(x)/alphaX; %compute moving target
Ftarget = ddytg -((1-x).^2).*(alphaY*(betaY*(yta-ytg)+(goalV-dytg)));
%Ftarget = ddytg -((1-x).^2).*(alphaY*(betaY*(goal-ytg)-dytg));



%% Computing weights
%s = x.*(goal-init);
s = x;
psiTra = psi;
psiDiag = full(sparse(1:numel(psiTra), repmat(1:size(psiTra,1),1,size(psiTra,2)), psiTra(:)));
midMatrix = reshape(psiDiag', [1000,1000,size(psi,2)]);

w = zeros(1,size(psi,2));
for ii = 1:size(psi,2)
    num = (s*midMatrix(:,:,ii)*Ftarget');
    den = (s*midMatrix(:,:,ii)*s');
    w(ii) = num/den;
end
for kk=1:1000
    f(kk) = ((psi(kk,:)*w')/sum(psi(kk,:)))*x(kk);% updating forcing term
end
num
 figure(2032+figNum)
 plot(Ftarget,'k');
 hold on
 plot(f,'r--');
end