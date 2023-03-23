function [ output_matrix ] = dUp( input_matrix, dimension )
%dUP Enlarges the matrix dimension
%   Creates a square matrix of size dimension. The square matrix "input_matrix"
%   is copied on the upper left, the remaining entries are zero.

% Checks if "input_matrix" is square
assert(size(input_matrix,1) == size(input_matrix,2));

% Creates new square matrix
output_matrix = zeros(dimension, dimension);

% Get dimensionality of "input_matrix"
old_dimension = size(input_matrix,1);

% Copies "input_matrix" to newly initialized matrix. Enlarges it if necessary
output_matrix(1:old_dimension,1:old_dimension) = input_matrix;

end

