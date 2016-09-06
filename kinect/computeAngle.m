function rad = computeAngle(x,y)
%% compute angle: calculate the angle beetween two vector; 
% returned angle is in radians
% input: two vectors;
% output: the angle in radians.
%rad = acos(dot(a/norm(a),b/norm(b)));

%rad = atan2(norm(cross(x,y)), dot(x,y));
nx = x/norm(x);
ny = y/norm(y);
%norm(nx - ny) / norm(nx + ny)
%rad = 2 * atan(norm(x*norm(y) - norm(x)*y) / norm(x * norm(y) + norm(x) * y))
rad = 2 * atan(norm(nx - ny) / norm(nx + ny));
