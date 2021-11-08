%% test the sum_parameters function
a = 3;
b = 5;
c = sum_parameters(a, b)
%% test transpose_and_scale
vector_in = [2,3,4];
scale_by = 2;
vector_out = transpose_and_scale(vector_in, scale_by)
%% test path_symbolic
q_start = [0 0 0 0 0 0];
q_target = [pi/2 pi/2 pi/3 2*pi/3 -pi/2 -pi/3];
path = path_symbolic(q_start, q_target)
%% test return_UR5_object
UR5 = return_UR5_object()
%% test return_robot_object
label ='universalUR3';
robot = return_robot_object(label)
%% test return_evaluated_matrrices
result = return_evaluated_matrices(0.0, 1.0)