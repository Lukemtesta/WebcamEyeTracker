function [ P ] = Linear( p, q )

% Compute perspective matrix using 4-point homography

if ( size(p,2) ~= 2 || size(q,2) ~= 2 )
    P = [];
    return;
end

num = p(2,1)*q(2,2) - q(2,1)*p(2,2);
den = q(2,1)*p(1,1) - p(1,1)*p(2,2);
c = num/den;
theta = acos(c);

a = ( q(1,1) - b*p(2,1) ) / p(1,1);

P = P * ( 1 / P(3,3) );  % Normalise Matrix

end



