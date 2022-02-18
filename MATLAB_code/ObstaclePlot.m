function ObstaclePlot(x_lim,y_lim,SafeDis,FObsPos,UObsPos)
if nargin < 5
    UObsPos = [];
end

num = size(FObsPos,2);
Obs = [FObsPos];
Circle = zeros(num,2);

num_2 = size(UObsPos,2);
Obs_2 = [UObsPos];
Circle2 = zeros(num,2);

for n = 1:num
    Circle(n,:) = Obs(:,n)';
end

for n = 1:num_2
    Circle2(n,:) = Obs_2(:,n)';
end
figure
hold on
xlim(x_lim);
ylim(y_lim);

for i = 1 :num
    rectangle('Curvature', [1 1],'Position',[Circle(i,1:2)-SafeDis/2,SafeDis,SafeDis],'facecolor','k')
    rectangle('Curvature', [1 1],'Position',[Circle(i,1:2)-SafeDis,2*SafeDis,2*SafeDis])
end

for i = 1 :num_2
    rectangle('Curvature', [1 1],'Position',[Circle2(i,1:2)-SafeDis/2,SafeDis,SafeDis],'facecolor','m','edgecolor','m')
    rectangle('Curvature', [1 1],'Position',[Circle2(i,1:2)-SafeDis,2*SafeDis,2*SafeDis],'edgecolor','m')
end
    
end

