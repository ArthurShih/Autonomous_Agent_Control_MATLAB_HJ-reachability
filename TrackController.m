function [x_re,y_re,th_re,u,t_b,collision_loc,flag] = TrackController(Init,Goal,dt,th,...
        radius,F_obs,D_obs_x,D_obs_y,U_obs,t_s)
%% Initialization     
x = Init(1);
y = Init(2);
timer = t_s;
flag = 0;

xt = Goal(1);
yt = Goal(2);

kp = 10;    
ki = 0.1;
kd = 0.1;

error = 0;
error_p = 0;
dis = sqrt((x-xt)^2 + (y-yt)^2);

x_re = [x];
y_re = [y];
th_re = [th];
n = 1;
u_lim = deg2rad(10);

dis_m = norm(Init - Goal);


L = 1;
u = 0;
collision_loc = [];

%% PID
while  (dis(n)^2 > 1e-2)
    th_t = atan2( (yt - y),(xt - x));
    th_e = th - th_t;
    error = error + th_e;
    u = -kp * (th_e) + ki * error + kd *(th_e - error_p);
    error_p = th_e;
    
    dis(n+1) = sqrt((x-xt)^2 + (y-yt)^2);
    v =  1;
    vr = v + u * L/2;
    vl = v - u * L/2;
    
    %%% process model: update
    v = (vl + vr)/2;
    u = (vr - vl)/L;
    x = x + v * cos(th)*dt;
    y = y + v * sin(th)*dt;
    if abs(u * dt) > u_lim
        angle = sign(u * dt) * u_lim;
    else
        angle = u * dt;
    end
    th = th + angle ;
    
    % Detection
    D_obs = [D_obs_x(timer,:);D_obs_y(timer,:)];
    obs_detection(timer,:) = detection([x,y],radius,F_obs,D_obs,U_obs);
    loc = find(obs_detection(timer,:) == 1);
    if ~isempty(loc)
        flag = 1; % collision signal
        collision_loc = loc;
        
        break
    else
        collision_loc = [];
    end
    
    
    
    x_re = [x_re;x];
    y_re = [y_re;y];
    th_re = [th_re;th];
    n = n + 1;
    timer = timer + 1;

    if norm([x y] - [xt yt]) >= dis_m
        break
    end
    if n > 800
        error('!')
    end
%     plot(x_re,y_re,'rs')
end
 
t_b = timer;
end