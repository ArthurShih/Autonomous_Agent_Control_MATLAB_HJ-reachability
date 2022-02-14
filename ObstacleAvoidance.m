function [path_new,path_RRT] = ObstacleAvoidance(Init,Target,m,p_angle,radius,collision_loc,v,F_obs,D_obs,U_obs)
% input should be the value at time k
% init is Loc(k) or we should call follower(end)
% target is Loc(k+1) or the related regions
path_RRT = [];
t_g = m/v;
if  t_g > 15
    t_g = 15;
end
% F_obs_traj = [];
% D_obs_traj = [];
% U_obs_traj = [];
Num_F = size(F_obs,2);
Num_D = size(D_obs,2);
Num_U = size(U_obs,2);

%  for k = 1:length(collision_loc) % find which types of obs is detected
%     if collision_loc(k) <= Num_F % Fixed Obs collision 
%         F_obs_traj = [F_obs_traj,F_obs(:,collision_loc(k))];
%     end
%     if collision_loc(k) <= (Num_F + Num_D) && collision_loc(k) > Num_F % Dynamic Obs collision
%         renum = collision_loc(k) - Num_F;
%         D_obs_traj = [D_obs_traj,D_obs(:,renum)];
%     end
%     if collision_loc(k) <= (Num_F + Num_D + Num_U) && collision_loc(k) > (Num_F + Num_D) % Unknown Obs collision
%         renum = collision_loc(k) - (Num_F + Num_D);
%         U_obs_traj = [U_obs_traj,U_obs(:,renum)];
%     end
%  end

    r = 10;
    path_n = [];
    [path_n,value] = HJ_avoid_obs(collision_loc,Init,Target,p_angle,r,t_g,v,radius,F_obs,...
            Num_F,D_obs,Num_D,U_obs,Num_U);

q = 1; 
qq = 1;
while value > 0
    if t_g + qq > 10
        path_n = [];
        [path_n,value] = HJ_avoid_obs(collision_loc,Init,Target,p_angle,r,t_g,v,radius - q,F_obs,...
                    Num_F,D_obs,Num_D,U_obs,Num_U);
        q = q + 1;
    else
        path_n = [];
        [path_n,value] = HJ_avoid_obs(collision_loc,Init,Target,p_angle,r,t_g + qq ,v,radius - q,F_obs,...
                    Num_F,D_obs,Num_D,U_obs,Num_U);
        qq = qq + 2;   
    end
    if q > 4 || t_g + qq > 20
        error('The value is still larger than zero')
    end

end 

dis_i = sqrt((Init(1) - Target(1))^2 + (Init(2)  - Target(2))^2) - 0.8 * r;
for a = 1:length(path_n)
    dis = sqrt((path_n(1,a) - Target(1))^2 + (path_n(2,a) - Target(2))^2);
    if dis <=  dis_i
        break
    end
end
path_new = path_n(:,1:a);

end