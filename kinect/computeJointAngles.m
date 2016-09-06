function angles = computeJointAngles(links)
% angles = computeJointAngles(links) computes angles of each joint of the
% skeleton; input matrix "links" is a matrix: each row is a vector defining
% a link. The order of these links is: head, torso, leftArm(shoulder,upper,lower),
% rightArm(shoulder, upper, lower). Output is a vector containing each
% computed angles; the order is: leftShoulderPitch, leftShoulderRoll,
% leftElbowRoll, leftElbowYaw


leftUpperArm = links(4,:);
leftShoulder = links(3,:);
leftLowerArm = links(5,:);
upperArmProj = leftUpperArm;
upperArmProj(2) = 0;
angles(1) = computeAngle(leftUpperArm, upperArmProj); %lshoulderPitch
if(leftUpperArm(2)>leftShoulder(2))
    angles(1) = -angles(1);
end

upperArmProj(2) = leftUpperArm(2);
upperArmProj(1) = 0;
angles(2) = computeAngle(leftUpperArm, upperArmProj); %lshoulderRoll


angles(3)= -computeAngle(leftUpperArm, leftLowerArm); %leRoll OK


normalToPlane = cross(leftUpperArm,leftLowerArm);
upperArmProj(1) = leftUpperArm(1);
upperArmProj(2) = 0;
normalToVerticalPlane = cross(leftUpperArm,upperArmProj);
angles(4) = computeAngle(normalToPlane, normalToVerticalPlane); %leYaw
angles(4)*180/pi


%%Old angle.
%y(1,zz) = pi/2 - computeAngle(leftUpperArm, torso); %lsPitch
%y(2,zz) = computeAngle(leftUpperArm, leftShoulder)-pi/2; %lsRoll
%y(3,zz) = computeAngle(leftUpperArm, leftLowerArm); %leRoll OOK