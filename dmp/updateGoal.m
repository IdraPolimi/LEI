function [goal, dgoal] = updateGoal(newGoal,goal,dgoal, dt,alphaG)
%

ddgoal = alphaG*(alphaG/4*(newGoal-goal)- dgoal);
goal = goal + dt*alphaG*dgoal;
dgoal = dgoal + dt*ddgoal*alphaG;


%lphaY*(betaY*(yta-y)+(goalV-dy)