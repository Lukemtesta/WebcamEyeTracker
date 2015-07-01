function [ q ] = ProjectionMap( P, p )

% Projection map point p with matrix P to position q

q = P * p';
alpha = 1 / q(3);
q = q*alpha;  

end

