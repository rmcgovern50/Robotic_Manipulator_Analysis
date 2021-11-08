function [vector_out] = transpose_and_scale(vector_in,scale_factor)
%TRANSPOSE_AND_SCALE Summary of this function goes here
%   transpose and scale an arbitrary vector returning the result
%transposed_vector = scale_factor.*transpose(vector_in)
vector_out = scale_factor.*transpose(vector_in)%scale_by*transpose(vector_in)
end

