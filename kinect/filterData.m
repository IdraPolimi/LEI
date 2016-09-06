function [filtData,error] = filterData(joints,errors,a)
error = zeros(size(joints,1),2);
filtData = zeros(45,1);

for ii = 1:size(joints,1)
    [filtData(ii), tempEX] = brown(joints(ii,end),joints(ii,2), joints(ii,1), errors(ii,2), errors(ii,1),a);
    error(ii,:) = [errors(ii,2),tempEX];
end
