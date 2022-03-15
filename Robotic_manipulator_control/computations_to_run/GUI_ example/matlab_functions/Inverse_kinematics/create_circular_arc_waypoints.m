function waypoints = create_circular_arc_waypoints()
%% define points
P1 = [0.5 0.5 0];
P3 = [-0.5 0.0 0];

%% do maths for a linear x_y equation and save points

wp_1 =  P1;
wp_n = P3;
number_of_steps = 50;

steps = (wp_n(1) - wp_1(1))/(number_of_steps-1);

y_val = [];
x_val = [];

for x = wp_1(1):steps:wp_n(1)
    x_val(end+1) = x;
    y_val(end+1) = x+0.5;
end

y_val;

%% do maths for the circle arc off the line
syms x y z a b c r

sphere = eq(r^2, (x-a)^2 + (y-b)^2 + (z-c)^2);
xy_line = eq(y, x);
z_equals = solve(sphere, z);


arc_equation = subs(z_equals, y, x);

arc_equation2 = subs(arc_equation(1), [r^2 a b c], [0.5 0 0 0]);

arc_equation_z_x = eq(z, arc_equation2)

%% sub into arc equation

z_val = [];

for x = wp_1(1):steps:wp_n(1)
    z_val(end+1) = subs(arc_equation2, x);
end

x_col = transpose(x_val);
y_val = transpose(y_val);
z_val = transpose(z_val);
waypoints = [x_col, y_val, z_val];

end