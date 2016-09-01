close all
clear w
clear ii

%% overriding params
tau = 1; %temporal scaling (=1 == same time for learning)
T = timeScale/(tau*dt); %duration of the action

%% computing time
x = ones(1,T);
for ii = 2:T
    x(ii) = canonicalSystem(x(ii-1),dt,alphaX,tau);
end

%% computing centers and variance of gaussians
[c,rho] = regModelParam(x,T,tPercentage,basisNumber,alphaX);
psi = regModel(x,c,rho,basisNumber,0);

%% learning the demonstrated trajectory (computing the weights for each kernel)
%each row of w corresponds to a gld
w = zeros(gdl,basisNumber);
for ii = 1:gdl
    disp(['Learning trajectory for ', ii, 'gld...']);
    w(ii,:) = learnTrajectory(ytg(ii,:), dytg(ii,:), ddytg(ii,:), alphaY, betaY, psi, x, alphaX,ii);
    disp(['Trajectory learned for ', ii,'gld!']);
end
