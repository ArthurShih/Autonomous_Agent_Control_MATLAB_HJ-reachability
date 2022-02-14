function Loc = AgentDynamic(t,path_RRT,LenSeg,vel)
% Parameters setting
D = vel * t;
px = path_RRT(:,1);
py = path_RRT(:,2);

% Judge whether the tracking path is out of range 
if D >= sum(LenSeg)
    Loc = [];
    return
end

% Find which path segment located
rge = zeros(1,2);
for n = 1:length(LenSeg)
    D = D - LenSeg(n);
    if D <= 0 
        rge = [n,n+1];
        D = D + LenSeg(n);
        break
    end
end

% Find the position 
p_local = [px(rge(2)) - px(rge(1)),py(rge(2)) - py(rge(1))];

if  isempty(find(abs(p_local) <= 1e-3,1))
    if p_local(1) > 0
        theta = atan((py(rge(2)) - py(rge(1)))/(px(rge(2)) - px(rge(1))));
            Loc(1) = px(rge(1)) + D * cos(theta);
            Loc(2) = py(rge(1)) + D * sin(theta);% Q = 1 & 2
    else
        theta = pi - atan((py(rge(2)) - py(rge(1)))/(px(rge(2)) - px(rge(1))));
            Loc(1) = px(rge(1)) + D * cos(theta);
            Loc(2) = py(rge(1)) - D * sin(theta); % Q = 3 & 4
    end
    
else
    if abs(p_local(1)) <= 1e-3
        Loc = [px(rge(1)),py(rge(1))] + [0, D];
    else
        Loc = [px(rge(1)),py(rge(1))] + [D, 0];
    end
end

end