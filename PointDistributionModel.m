
% Script generates a PDM for a single circle
% Dataset stored in PDM_Distribution

%S_Distribution % 0.01 Resolution
%RadiusX_Distribution % 0.01 Resolution
%RadiusY_Distribution % 0.01 Resolution


%% Search Parameters

Range.RLower = 0;
Range.RUpper = 1;
Range.RxLower = 0;
Range.RxUpper = 1*pi/180;   % 0.1 Degrees resolution
Range.RyLower = 0;
Range.RyUpper = 1*pi/180;   % 0.1 Degrees resolution
Range.SLower = 0;
Range.SUpper = 1;         

CirclePoints = 200;
Iterations = 10;
interval = 2*pi/CirclePoints;

%% Generate Dataset

Angles = [0:interval:(2*pi - interval)];
Template.pts = [ sin(Angles); cos(Angles); zeros( 1, size(Angles,2) ) ];

%% Build PDM

data = [];
interval = Range.SUpper/Iterations;
for i=Range.SLower:interval:(Range.SUpper - interval)
    
    CircleA = TransformCircle( Template.pts, ...
                   0, [1; 0; 0], [0; 0; 0], i );
             
    scatter( CircleA.pts.Transformed(1,:), CircleA.pts.Transformed(2,:), 'r')
    axis([-1 1 -1 1])
    data = [data, CircleA.pts.Transformed];
    k=waitforbuttonpress
end

e = Eigen_Build( data(1,:)' );
Distx = e.vct(1:200,1);
e = Eigen_Build( data(2,:)' );
Disty = e.vct(1:200,1);

S_Distribution = CircleA.pts.Transformed;

p = Template.pts(1:2,:);
p = p - S_Distribution*0.01;
scatter( p(1,:), p(2,:), 'r')
axis([-1 1 -1 1])

