deltas = [.1,.1,.1,.1]/100;

A = zeros(4);
% wh whd th thd
init_state = [0,0,0,0];
init_u = 0;

for i = 1:4
    delta = deltas(i);
    state = init_state;
    state(i) = state(i) - delta;
    [wh_new whd_new whdd_new th_new thd_new thdd_new] = model(state(1), state(2), state(3), state(4), u);
    res_low = [wh_new, whd_new, th_new, thd_new];

    state = init_state;
    state(i) = state(i) + delta;
    [wh_new whd_new whdd_new th_new thd_new thdd_new] = model(state(1), state(2), state(3), state(4), u);
    res_high = [wh_new, whd_new, th_new, thd_new];

    deriv = (res_high - res_low) / (2*delta);
    A(:,i) = deriv;
end

delta = .1;
state = init_state;
u = init_u - delta;
[wh_new whd_new whdd_new th_new thd_new thdd_new] = model(state(1), state(2), state(3), state(4), u);
res_low = [wh_new, whd_new, th_new, thd_new];

state = init_state;
u = init_u + delta;
[wh_new whd_new whdd_new th_new thd_new thdd_new] = model(state(1), state(2), state(3), state(4), u);
res_high = [wh_new, whd_new, th_new, thd_new];

B = (res_high - res_low) / (2*delta);

A
B