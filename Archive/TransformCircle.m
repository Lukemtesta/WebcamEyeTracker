% Apply affine transform on point and return class containing
% transform information

function Object = TransformCircle( pts, Inertia, AxisAngle, T, S )

% Set Object Contents

Object.pts.Original = pts;
Object.Translation = T;
Object.Scale = S;
Object.AxisAngle.Amount = Inertia;
Object.AxisAngle.About = AxisAngle;

temp = dot( repmat(AxisAngle, 1, size(pts,2) ), pts);
w = [temp; temp; temp];

pts = ( cos(Inertia)*pts ) + ( sin(Inertia)* ...
            cross( repmat(AxisAngle, 1, size(pts,2) ), pts) ) ...
                 + ( (1 - cos( Inertia ) ) * ...
                       w ...
                            .* repmat(AxisAngle, 1, size(pts,2) ) );
pts = pts*S + repmat(T, 1, size(pts,2) ); 

Object.pts.Transformed = pts(1:2,:);
    
end

