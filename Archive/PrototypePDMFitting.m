% Function generates a circle and uses 4-pt homography
% to derive the projection matrix between keypoints.
% This is followed with gradient descent to locate the
% circle parameters

% img = RGB image
% PDM = PDM model

function Circle = DetectCircle( imname, PDM )

img = edge( rgb2gray( imread(imname) ) );
figure(1), imshow(imname), title('RGB: Test Circle')
figure(2), imshow(img), title('Canny: Test Circle')

interval = 2*pi/PDM.N;                                              % Make Circle
Angles = [0:interval:(2*pi - interval)];
Template.pts = [ sin(Angles); cos(Angles); ];
            
val = [21, 121];                                                    % Select keypoints
kp = [ Template.pts( :, val ) ];

[row col] = find( img == 1 );                                       % Edge Locations
indices = [round( rand(1,2)*(size(row,1) - 2) + 1 ) ; ...
                 round( rand(1,2)*(size(row,1) - 2) + 1) ];

kp_img = [row( indices(1,:) )'; col( indices(1,:) )'; ];

T = mean(kp_img')' - mean(kp')';                                    % Compute [Tx, Ty]
q = kp_img - repmat(T,1,2);

ptsn = kp(:,1) / norm(kp(:,1));                                     % Compute Rz
kp_imgn = q(:,1) / norm(q(:,1));

Rz = atan2( kp_imgn(2,:), kp_imgn(1,:) ) - ...
                atan2( ptsn(2,:), ptsn(1,:) );

M = [ cos(Rz) -sin(Rz) T(1); sin(Rz) ...                            % Compute M
            cos(Rz) T(2); 0 0 1 ]

FBTE = ForwardBackwardTransferError( M, kp, kp_img )                % Compute FBTE

pts = [Template.pts; ones(1,size(Template.pts,2))];                 % Render Circle
pts = M*pts;
%pts = M*pts;
kp_draw = M*[kp; 1 1];
%pts = M*[kp; 1 1]
figure(3), scatter( pts(1,:), pts(2,:), 'r.')
figure(3), hold on, scatter( img(1,:), img(2,:), 'bx')
figure(3), scatter( kp_img(1,:), kp_img(2,:), 'g^')
figure(3), scatter( kp_draw(1,:), kp_draw(2,:), 'mo')
hold off, legend('Template', 'Target',  ... 
                    'kp: Template', 'kp: Target')
        

end

