close all
clear x

clear psi
clear y
clear dy
clear ddy


clear plotX
clear plotY
clear plotdY
clear plotddY
clear plotX
clear plotPSI
clear plotf
clear plotTAU
clear plotddYT
clear plotYNEW

clear plotE

%%
vrep=remApi('remoteApi');
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart(IPADDRESS, PORT, WAIT_UNTIL_CONNECTED, DO_NOT_RECONNECT_ONCE_DISCONNECTED, TIME_OUT_IN_MSEC, COMM_THREAD_CYCLE_IN_MS);

if (clientID>-1)
    % reading handles number
    for jj = 1:gdl
        [~, jointsHandle(jj)] = vrep.simxGetObjectHandle(clientID, cell2mat(jointsName(jj)), vrep.simx_opmode_oneshot_wait);
        tempLimit = jointsLimits(strcmp({jointsLimits.name},cell2mat(jointsName(jj))));
        upperLimits(jj) = tempLimit.upperLimit;
        lowerLimits(jj) = tempLimit.lowerLimit;
    end
    vrep.simxSynchronous(clientID,false);
    if(~vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot))
        disp('Error in starting simulation');
        return
    end
    for jj = 1:gdl
        [ret(jj), ~] = vrep.simxGetJointPosition(clientID,jointsHandle(jj),vrep.simx_opmode_streaming);
    end
    while (any(ret~=vrep.simx_return_ok))
        for jj = 1:gdl
            [ret(jj), positions(jj)] = vrep.simxGetJointPosition(clientID,jointsHandle(jj),vrep.simx_opmode_buffer);
            %positions(jj) = 0;
        end
    end
    
    %% overriding params
    plotGraph = true;
    
    goal = ytg(:,end); %goal coordinates for each dof
    dgoal = zeros(gdl,1); %goal speed for each dof
    %yInit = positions;
    yInit = ytg(:,1); % initial coordinates for each dof
    dyInit = dytg(:,1); % initial speed for each dof
    goalV = dytg(:,end); % final speed of each dof
    tau = 1; %gain term for speed of movement;
    
    %% computing centers and variance of gaussians
    T = timeScale/(tau*dt);
    x = ones(1,T);
    for ii = 2:T
        x(ii) = canonicalSystem(x(ii-1),dt,alphaX,tau);
    end
    
    [c,rho] = regModelParam(x,T,tPercentage,basisNumber,alphaX);
    
    %% main loop
    xEnd = x(end);
    x = 1; %resetting phase variable for starting the movement
    dy = dyInit;
    y = yInit
    newGoal = goal;
    error = zeros(gdl,1);
    yNew = y;
    ii = 1;
    %%set robot starting position
    
    moveRobot(jointsHandle, y, upperLimits, lowerLimits,  vrep, clientID,dt);
    pause
    %for ii = 1:T
        while (x> 0.001)
        ii
        psi(ii,:) = regModel(x,c,rho,basisNumber,1);
        for jj = 1:gdl
            %f = ((psi*w(jj,:)')/sum(psi))*x*(goal(jj)-yInit(jj))*scale;% updating forcing term
            f(jj) = ((psi(ii,:)*w(jj,:)')/sum(psi(ii,:)))*x*scale;% updating forcing term
            [y(jj), dy(jj), ddy(jj),ta(jj),ddyT(jj)] = transformationSystem(alphaY, betaY, goal(jj), goalV(jj), dt, dy(jj), y(jj), f(jj), tau,x,xEnd, alphaX, error(jj)); %computing trajectory
            %yReal(jj) = dummyRobot(y(jj), yNew(jj),dt,ii);
            %error(jj) = (yNew(jj) - y(jj))^2;
            [goal(jj),dgoal(jj)] = updateGoal(newGoal(jj),goal(jj),dgoal(jj), dt,alphaG);
        end
        
        yReal = moveRobot(jointsHandle, y, upperLimits, lowerLimits,  vrep, clientID, dt);
        %yReal = dummyRobot(y(jj), yNew,dt,ii);
        for jj = 1:gdl
            error(jj) = (yReal(jj) - y(jj))^2;
        end
        tau = 1/(exp(100*max(error)));
        %tau = 1;
        x = canonicalSystem(x,dt,alphaX, tau);
        
        if (plotGraph)
            plotE(:,ii) = error;
            plotf(:,ii) = f;
            plotTAU(ii) = tau;
            plotX(ii) = x;
            plotY(:,ii) = y;
            plotdY(:,ii) = dy;
            plotddY(:,ii) = ddy;
            plotddYT(:,ii) = ddyT;
            plotYReal(:,ii) = yReal;
        end
        ii = ii+1;
    end
    for jj = 1:gdl
        vrep.simxGetJointPosition(clientID,jointsHandle(jj),vrep.simx_opmode_discontinue);
    end
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
    vrep.simxFinish(clientID);
    %% plotting
    if(plotGraph)
        figure(1)
        clf
        plot(plotX)
        title('Time (cycle)');
        
        figure (2)
        clf
        plot(plotTAU)
        title('TAU');
        
        figure(3)
        clf
        hold on
        for ii = 1:basisNumber
            plot(psi(:,ii));
        end
        title('Basis functions');
        
        for jj = 1:gdl
            graphName = jj*10;
            figure(graphName)
            clf
            plot(plotY(jj,:),'r')
            hold on
            plot(ytg(jj,:),'k')
            title('Output trajectory');
            plot(plotYReal(jj,:),'b--')
            
            figure(graphName+1)
            clf
            plot(plotdY(jj,:),'r')
            hold on
            plot(dytg(jj,:),'k')
            title('Output Velocity');
            
            figure(graphName+2)
            clf
            plot(plotddY(jj,:),'r')
            hold on
            plot(ddytg(jj,:),'k')
            title('Output Acceleration');
            
            figure (4)
            hold on
            plot(plotE(jj,:),'r')
            title('Error');
            hold off
        end
    end
else
    disp('Failed connecting to remote API server');
end
