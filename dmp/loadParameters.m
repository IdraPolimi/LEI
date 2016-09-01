%% MODEL PARAM Run to load model and vrep parameters.

tPercentage = 0.90; %duration of influence of regression model wrt total movement duration
timeScale = 1; %scale for total duration of action; the bigger, the longer. *DO NOT CHANGE
dt = 0.001; %integration time *DO NOT CHANGE
alphaX = 7.5; %gain term for "time", change the shape of time; set so that x->0 in T timestamp. *DO NOT CHANGE
tau = 1; %deafult gain term for the speed of the action; > 1 --> speed up movements, <1 --> slow down moevments.
alphaY = 10; %gain term for trajectory acceleration *DO NOT CHANGE
betaY = alphaY*tau/4; %gain term for trajectory acceleration (set equal alphaY/4 for critical dampening) *DO NOT CHANGE
basisNumber = 10; %number of basis functions
gdl = 4; %number of degrees of freedom
alphaGmin = 3.5; % min gain term for goal updating
alphaGmax = 7.5;  % max gain term for goal updating, do not exceed to avoid spikes in acc and speed
alphaG = alphaGmin;
dgoal = 0; %initial speed for goal updating
scale = 1; %scaling factor for dmp
alphaE = 1; %gain term for error
