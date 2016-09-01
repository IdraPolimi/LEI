function feasible = checkFeasibility(angularValue, upperLimit, lowerLimit)
%% checkFeasibility: determine if a joint position is feasible or not. 
%
%      feasible = checkFeasibility(angularValue, upperLimit, lowerLimit) 
%      return if angularValue (in degree) is within upperLimit (in degree) 
%      and lowerLimit (in degree)
    %angularValue
    %upperLimit
    %lowerLimit
    
    feasible = angularValue <= upperLimit && angularValue >= lowerLimit;
end