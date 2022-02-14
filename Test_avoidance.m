%% Should we compute the trajectory?
% function [traj,dt,value] = Test_avoidance(detect,Init_pos,target_pos,radius_tar,t_gap,v,radius_obs,F_obs,Num_F,D_obs,Num_D,U_obs,Num_U)
Init_pos = follower(end,:);
target_pos = target_pt;
radius_tar = 3;
t_gap = t_g;
radius_obs = radius;
%% Should we compute the trajectory?
compTraj = true;

%% Grid
grid_min = [Init_pos(1)-1;Init_pos(2)-1; -pi]; % Lower corner of computation domain
grid_max = [Init_pos(1)+14; Init_pos(2)+14; pi];    % Upper corner of computation domain
N = [41; 41; 41];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set
R = radius_tar;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
data0 = shapeCylinder(g,3,[target_pos(1); target_pos(2); 0], R);
% also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector
t0 = 0;
tMax = t_gap;
dt = 0.1;
tau = t0:dt:tMax;

%% problem parameters

% input bounds
speed = v;
wMax = 1;
% do dStep1 here

% control trying to min or max value function?
uMode = 'min';
% do dStep2 here 

%% Pack problem parameters

% Define dynamic system
% obj = DubinsCar(x, wMax, speed, dMax)-- (state: [xpos; ypos], angle max(or min), speed, disturbance bounds)
dCar = DubinsCar([target_pos(1), target_pos(2), 0], wMax, speed); %do dStep3 here What does the states here means?

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'high'; %set accuracy  [low, medium, high, veryHigh]
schemeData.uMode = uMode;
%do dStep4 here

%% additive random noise

%% If you have obstacles, compute them here 
g_func1 = inline('60 + 5.*sin(0.2 .* x1)'); % should be changed once the dynamic function changes
g_func2 = inline('80 + 5.*sin(0.2 .* x2)');
% Should be rewrite for different situations 
Num_o = length(detect);
if Num_o == 1
    union_time = 0;
else
    union_time = round(Num_o/2);
end
Dyn_detect = find(detect > Num_F && detect <= (Num_F + Num_D),1);
if isempty(Dyn_detect) 
    flag = 0;
else
    flag = 1;
end


gDim = g.dim;
clns = repmat({':'}, 1, gDim);

for n = 1:length(detect)
    if flag == 0
        if detect(n) <= Num_F % Fix
            Tmp = shapeCylinder(g, 3, [F_obs(1,detect(n)); F_obs(2,detect(n)); 0], radius_obs);        
        else % Unk
            Tmp = shapeCylinder(g, 3, [U_obs(1,detect(n)-(Num_F + Num_D)); U_obs(2,detect(n)-(Num_F + Num_D)); 0], radius_obs);
        end
    else % Time seq needed
        if detect(n) <= Num_F % Fix
            for ii = 1:length(tau)
                Tmp(clns{:}, ii) = shapeCylinder(g, 3, [F_obs(1,detect(n)); F_obs(2,detect(n)); 0], radius_obs);
            end
        elseif detect(n) <= (Num_F + Num_D) && detect(n) > Num_F % Dyn
            for ii = 1: length(tau)
                eval(['Obs_traj(ii,:) = [D_obs(1,n) + dt * (ii-1); g_func',...
                    num2str(detect(n) - Num_F),'(D_obs(1,n) + dt * (ii-1))];'])
                Tmp(clns{:}, ii) = shapeCylinder(g, 3, [Obs_traj(ii,1);Obs_traj(ii,2); 0], radius_obs);
            end
        else % Unk
            for ii = 1:length(tau)
                Tmp(clns{:}, ii) = shapeCylinder(g, 3, [U_obs(1,detect(n)-(Num_F + Num_D)); U_obs(2,detect(n)-(Num_F + Num_D)); 0], radius_obs);
            end
        end
    end
    if union_time == 0 % Not shapeunion needed
        union_tmp = Tmp;
    else % ShapeUnion needed
        if n == 1 % first union obs
            union_tmp = Tmp;
        else
            union_tmp  =  shapeUnion(union_tmp,Tmp);
        end
    end
end
HJIextraArgs.obstacleFunction = flip(union_tmp);

%% Compute value function
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'zero', HJIextraArgs);

%% Compute optimal trajectory from some initial state
if compTraj
  
  %set the initial state (Start point for the trajectory)
  xinit = [Init_pos(1),Init_pos(2), pi/4];
  
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g,data(:,:,:,end),xinit);
  
  if value <= 0 %if initial state is in BRS/BRT
    % find optimal trajectory
    
    dCar.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = uMode; %set if control wants to min or max
    TrajextraArgs.dMode = 'max';
    
    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1 0]; 
    
    %flip data time points so we start from the beginning of time
    dataTraj = flip(data,4);
    
    [traj, traj_tau] = ...
      computeOptTraj(g, dataTraj, tau2, dCar, TrajextraArgs);
  figure(4)

    % add the target set to that
    [g2D, data2D] = proj(g, data0, [0 0 1]);
    visSetIm(g2D, data2D, 'green');
    hold on
    [g2D_of,data2D_of] = proj(g,Tmp,[0 0 1]);
    visSetIm(g2D_of,data2D_of,'black');
    for n = 1:length(traj(1,:))
        plot(traj(1,n), traj(2,n),'bo','MarkerSize',5)
        plot(Obs_traj(n,1),Obs_traj(n,2))
        xlim([Init_pos(1)-1 Init_pos(1)+14])
        ylim([Init_pos(2)-1 Init_pos(2)+14])
        pause(0.01)
    end
%     figure(6)
%     clf
%     h = visSetIm(g, data(:,:,:,end));
%     h.FaceAlpha = .3;
%     hold on
%     s = scatter3(xinit(1), xinit(2), xinit(3));
%     s.SizeData = 70;
%     title('The reachable set at the end and x_init')
%     hold off
%     [~,c] = size(traj);
%     
%     figure(4)
%     traj1 = flip(obs_lower);
%     traj2 = flip(obs_upper);
%     % add the target set to that
%     [g2D, data2D] = proj(g, data0, [0 0 1]);
%     visSetIm(g2D, data2D, 'green');
% 
%     hold on
%     
%     plot(traj1(1,1),traj1(1,2),'r+','MarkerSize',50,'LineWidth',3)
%     plot(traj2(1,1),traj2(1,2),'y+','MarkerSize',50,'LineWidth',3)
% 
% 
%     for n = 1:c
%         plot(traj(1,n), traj(2,n),'bo','MarkerSize',5)
%         plot(traj1(n,1),traj1(n,2),'ro','MarkerSize',30)
%         plot(traj2(n,1),traj2(n,2),'yo','MarkerSize',30)
%         xlim([-8 8])
%         ylim([-8 8])
%         pause(0.05)
%     end
%     title('2D projection of the trajectory & target set')
%     hold off
  else
      traj = [];
%     error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end
% end
