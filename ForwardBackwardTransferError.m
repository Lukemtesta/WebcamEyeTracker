% Computes the forward transfer error for 
% 2 sets of 2D points

function err = ForwardBackwardTransferError( M, p, q )

if size(p,1) ~= 2 || size(q,1) ~= 2
    error('Input points are not 2D')
    err = [];
end

if size(p,2) ~= size(q,2)
    error( 'Number of I/O pts does not match' )
    err = [];
end

p = [p; ones(1, size(p,2) ) ];
q = [q; ones(1, size(q,2) ) ];

err = sum( sum( (M*p - q) .^ 2 ) );
err = err + sum( sum( ( (inv(M)*q - p) .^ 2 ) ) );

end

