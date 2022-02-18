close all
clear
clc
%% Start node & Target node
s_node = [5,5];
t_node = [95,95];
x_lim = [0,100];
y_lim = [0,100];

%% Set obstacles 
% Fixed obstacles
% F_obs = [40,70,60;...
%          50,50,75];
F_obs = [40,80,60;...
         50,50,70];

% obstacle radius
radius = 10;

% Unkown fixed obstacles
U_obs = [20;20];

% Dynamic obstacles
dt = 0.1;
v = 1;
D_obs_start = [100,0;
              60,80];
D_obs_end = [0,100;
              60,80];
          
% Obstacle trajectory (time forward)
x1 = D_obs_start(1,1):-dt*0.5: D_obs_end(1,1);
x1 = x1';
y1c =  D_obs_start(2,1);
y1 =  y1c + 5*sin(0.2*x1);

x2 =  D_obs_start(1,2):dt*0.5: D_obs_end(1,2);
x2 = x2';
y2c = D_obs_start(2,2);
y2 = y2c + 5*sin(0.2*x2);


D_obs_x = [x1, x2];
D_obs_y = [y1, y2];


%% Path planning
% RRT_star path planning
stepsize = 5;
% tic
path_RRT = RRT_start(s_node,t_node,1.1*radius,F_obs(1,:),F_obs(2,:),stepsize);
% toc
%%% plot the path
% ObstaclePlot(x_lim,y_lim,radius,F_obs,U_obs)
% hold on
% plot(path_RRT(:,1),path_RRT(:,2))

%% Path tracking
% Path total length
Len = zeros(length(path_RRT)-1,1);
for i = 1: length(path_RRT)-1
    Len(i) = norm(path_RRT(i+1,1:2) - path_RRT(i,1:2));
end
% Path reference points
approximate_tau = floor(sum(Len)/v); % Define the number of reference pts   
for i = 1: approximate_tau
    Tmp = AgentDynamic(i,path_RRT,Len,v);
    if isempty(Tmp)
        break
    else
        Loc(i,:) = Tmp;
    end
end

%% Tracking controller
[follower, angle_change] = TrackandAvoid(Loc,v,radius,F_obs,D_obs_x,D_obs_y,U_obs,dt);

%% Plot
tau = length(follower);

myVideo=VideoWriter('Unknown_obs_test1128.avi');
myVideo.FrameRate=150;
open(myVideo);

ObstaclePlot(x_lim,y_lim,radius,F_obs,U_obs)
hold on
plot(Loc(:,1),Loc(:,2),'o')
pause(1)

for j = 1:tau
        plot(follower(j,1),follower(j,2),'ro')
        plot(x1(j),y1(j),'go','MarkerSize',20)
        plot(x2(j),y2(j),'yo','MarkerSize',20)
%         pause(0.005)
        temp=getframe(gcf);
        frame=temp.cdata;
        writeVideo(myVideo,frame);
end

hold off
close(myVideo)


figure
time = dt.*(1:tau);
plot(time,rad2deg(angle_change))
% figure(3)
% plot(U)

