% xx is [x body_angle ]
function plot_twip(xx, wheel_radius, length)
    
global first_time;
global l_pendulum;
global l_wheel;

if first_time
    first_time = false;
    f1 = figure(1);
    clf( f1 );
    l_pendulum = line( [xx(1) xx(1)+length*sin(xx(2))],[wheel_radius wheel_radius+length*cos(xx(2))] );
    l_pendulum.LineWidth = 5;
    l_wheel = line( [xx(1) xx(1)],[wheel_radius wheel_radius] );
    l_wheel.LineWidth = 5;
    l_wheel.Marker = 'o';
    l_wheel.MarkerSize = 50;
    axis([-8*length 8*length 0 2*length]);
    title( 'animation' );
    drawnow;
else
    l_pendulum.XData = [xx(1) xx(1)+length*sin(xx(2))];
    l_pendulum.YData = [wheel_radius wheel_radius+length*cos(xx(2))];
    l_wheel.XData = [xx(1) xx(1)];
    drawnow;
end
