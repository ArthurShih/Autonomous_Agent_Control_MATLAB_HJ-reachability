function array_D = detection(Loc, SafeDis,FObsPos,DObsPos,UObsPos)
if nargin < 4
    DObsPos = [];
end
if nargin < 5
    UObsPos = [];
end
% Location valid detection
if (Loc(1) < 0) || (Loc(2) < 0) || (Loc(1) > 100) || (Loc(2) > 100)
    error('The location of agent is out of the world!')
end

% Obstacle detection
num = size(FObsPos,2)+ size(DObsPos,2)+ size(UObsPos,2);
distance = zeros(num,1);
array_D = distance;
Obs = [FObsPos,DObsPos,UObsPos];

for n = 1:num
    distance(n) = sqrt((Loc(1) - Obs(1,n))^2 + (Loc(2) - Obs(2,n))^2);
    if distance(n) <= SafeDis
        array_D(n) = 1;
    end
end



end