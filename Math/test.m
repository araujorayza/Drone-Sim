Definitions;

%test to simplify the nonlinear model (aiming to use TS fuzzy)

simplify(M'*M)
simplify(-N*R'*M)

% What I came up with M'*dx = [-M'NR' 0;M 0]*x + [const;0]*v
% which doesn't make it easier