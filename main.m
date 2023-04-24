global first_time
global DT
first_time = true;

% settings
SAMPLES_PER_SECOND = 100;
DURATION = 2;
N = round(DURATION*SAMPLES_PER_SECOND);
DT = DURATION/N;

x_array = zeros(N,2); % states
xd_array = zeros(N,2); % velocities
xdd_array = zeros(N,2); % accelerations
u_array = zeros(N,1); % commands

% model states
mx_array = zeros(N,4); % states

% state is wh, whd, th, thd
x_array(1,:) = [0, .3];
xd_array(1,:) = [0, 0];

mx_array(1,:) = [x_array(1,1) xd_array(1,1) x_array(1,2) xd_array(1,2)];

% lqr matrices
Kc = [0.8981    5.4732  167.8600   16.2788];
% Kc = [0 0 200 0];
% Kc = [0 0 0 0];
A = [1 -.0182 0.0093 0;
     0 1.0158 0.0005 0.01;
     0 -1.82 .9303 0;
     0 1.5835 0.0498 1.000];
B = [-0.0003; 
     .0002;
     -.02903;
     .021];

for i = 2:N
    if i == 2 || mod(i,25) == 0
        [i, x_array(i-1,1), xd_array(i-1,1), x_array(i-1,2), xd_array(i-1,2), xdd_array(i-1,1), xdd_array(i-1,2)]
    end
    
    % compute control
    u = -Kc * [x_array(i-1,1); xd_array(i-1,1); x_array(i-1,2); xd_array(i-1,2)];
    u_array(i,:) = u;

    % compute accelerations
    [ wh_new whd_new whdd_new th_new thd_new thdd_new ] = model(x_array(i-1,1), xd_array(i-1,1), x_array(i-1,2), xd_array(i-1,2), u);
    x_array(i,:) = [wh_new th_new];
    xd_array(i,:) = [whd_new thd_new];
    xdd_array(i,:) = [whdd_new thdd_new];
    
    % compute model positions
    mx_array(i,:) = A * mx_array(i-1,:)' + B * u;
    
    if abs(x_array(i,2)) > 1
        x_array = x_array(1:i,:);
        xd_array = xd_array(1:i,:);
        xdd_array = xdd_array(1:i,:);
        u_array = u_array(1:i,:);
        mx_array = mx_array(1:i,:);
        break
    end    
end

figure(2)
plot(x_array(:,1));
title( 'wh' )

figure(3)
plot(x_array(:,2));
title( 'th' )

figure(4)
plot(xd_array(:,1));
title( 'whd' )

figure(5)
plot(xd_array(:,2));
title( 'thd' )

figure(6)
plot(u_array(:,1));
title( 'u' )

% figure(7)
% plot(xdd_array(:,1));
% title( 'whdd' )
% 
% figure(8)
% plot(xdd_array(:,2));
% title( 'thdd' )
% 
% figure(9)
% plot(mx_array(:,1));
% title( 'mwh' )
% 
% figure(10)
% plot(mx_array(:,3));
% title( 'mth' )
% 
% figure(11)
% plot(mx_array(:,2));
% title( 'mwhd' )
% 
% figure(12)
% plot(mx_array(:,4));
% title( 'mthd' )
