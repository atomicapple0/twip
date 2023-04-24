function [wh_new whd_new whdd_new th_new thd_new thdd_new] = model(wh, whd, th, thd, u)

DT = 1/100;

% compute accelerations
[whdd_new thdd_new] = twip2(wh, whd, th, thd, u);

% compute velocities
vels = [whd thd] + [whdd_new thdd_new]*DT;
whd_new = vels(1);
thd_new = vels(2);

% compute positions
pos = [wh th] + [whd_new thd_new]*DT;
wh_new = pos(1);
th_new = pos(2);