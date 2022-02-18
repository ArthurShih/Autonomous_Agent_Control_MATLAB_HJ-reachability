clear;clc
x_lim = [0,100];
y_lim = [0,100];
v = 1;

D_obs_start = [100,0;
              60,80];
D_obs_end = [0,100;
              60,80];
dis = floor(100*sqrt(2)/v);         
x1 = linspace(D_obs_start(1,1), D_obs_end(1,1),dis)'; 
a1 = flip(x1);
y1 = D_obs_start(2,1) + 5*sin(0.2*x1);
x2 = linspace(D_obs_start(1,2), D_obs_end(1,2),dis)';
y2 = D_obs_start(2,2) + 5*sin(0.2*x2);


figure(1)
for n = 1:dis
    plot(x1(n),y1(n),'ro',x2(n),y2(n),'bo',a1(n),a1(n),'yo');
    xlim(x_lim)
    ylim(y_lim)
    pause(0.05)
    hold on
end
