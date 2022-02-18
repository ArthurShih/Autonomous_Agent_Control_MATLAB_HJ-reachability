function [traj,value] = HJ_avoid_obs(detect,Init_pos,target_pos,p_angle,radius_tar,...
            t_gap,v,radius_obs,F_obs,Num_F,D_obs,Num_D,U_obs,Num_U)
%% Should we compute the trajectory?
compTraj = true;

%% Grid
grid_min = [Init_pos(1)-6;Init_pos(2)-6; -pi]; % Lower corner of computation domain
grid_max = [Init_pos(1)+20; Init_pos(2)+20; pi];    % Upper corner of computation domain
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
dCar = DubinsCar([target_pos(1), target_pos(2), p_angle], wMax, speed); %do dStep3 here What does the states here means?

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
    Dyn_detect = find(detect > Num_F && detect <= (Num_F + Num_D),1);

else
    union_time = round(Num_o/2);
    for n = 1:Num_o
        Dyn_detect = find(detect(n) > Num_F && detect(n) <= (Num_F + Num_D),1);
    end
end
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
            if detect(n) == Num_F + 1 % moving to left so need right side points
                for ii = 1: length(tau)
                    eval(['Tmp(clns{:}, ii) = shapeCylinder(g, 3, [D_obs(1,1) + dt * (ii-1); g_func',...
                        num2str(detect(n) - Num_F),'(D_obs(1,1) + dt * (ii-1)); 0], radius_obs);'])
                end
            else        % moving to right so need left side points
                for ii = 1: length(tau)
                    eval(['Tmp(clns{:}, ii) = shapeCylinder(g, 3, [D_obs(1,2) - dt * (ii-1); g_func',...
                        num2str(detect(n) - Num_F),'(D_obs(1,2) - dt * (ii-1)); 0], radius_obs);'])
                end
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
HJIextraArgs.obstacleFunction = union_tmp;

%% Compute value function
% HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update
HJIextraArgs.visualize.obstacleSet = 1;
HJIextraArgs.visualize.obstacleFunction = 1;    % what it means?
% uncomment if you want to see a 2D slice
HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; %plot x, y
HJIextraArgs.visualize.plotData.projpt = [0]; %project at theta = 0
HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'zero', HJIextraArgs);

%% Compute optimal trajectory from some initial state
if compTraj
  
  %set the initial state (Start point for the trajectory)
  xinit = [Init_pos(1),Init_pos(2), p_angle];
  
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g,data(:,:,:,end),xinit);
  
  if value <= 0 %if initial state is in BRS/BRT
    % find optimal trajectory
    
    dCar.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = uMode; %set if control wants to min or max
    TrajextraArgs.dMode = 'max';
%     TrajextraArgs.visualize = true; %show plot
%     TrajextraArgs.fig_num = 2; %figure number
    
    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1 0]; 
    
    %flip data time points so we start from the beginning of time
    dataTraj = flip(data,4);
    
    [traj, traj_tau] = ...
      computeOptTraj(g, dataTraj, tau2, dCar, TrajextraArgs);
  else
      traj = [];
%     error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end
end


