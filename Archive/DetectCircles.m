% Function generates a circle and uses 4-pt homography
% to derive the projection matrix between keypoints.
% This is followed with gradient descent to locate the
% circle parameters

% img = RGB image
% PDM = PDM model

function Circle = DetectCircle( imname, PDM, angle )

interval = 2*pi/PDM.N;                                                      % Make Circle
Angles = [0:interval:(2*pi - interval)];
Template.pts = [ sin(Angles); cos(Angles); ];
Template.pts = [ Template.pts - repmat([0.5; 0],1,200), Template.pts + repmat([0.5; 0],1,200) ]

%% The Hack Test Code

angle = angle*pi/180;
K = [cos(angle) -sin(angle) 50; sin(angle) cos(angle) 99;
        0 0 1];
img = Template.pts;
img = [img; ones(1,size(img,2))];
img = K*img;
img = img(1:2,:);
img(2,:) = img(2,:)*50;
img(1,:) = img(1,:)*24;

data = Template.pts;
scale = 1;
U(:,:,1) = [PDM.Sx, PDM.Sx];            % Converge Scale [Sx Sy]
U(:,:,2) = [PDM.Sy, PDM.Sy];            
U(:,:,3) = [-PDM.Sx, -PDM.Sx];
U(:,:,4) = [-PDM.Sy, -PDM.Sy];
U(:,:,5) = [PDM.RadiusX, -PDM.RadiusX]; % Converge Separation
U(:,:,6) = [-PDM.RadiusX, PDM.RadiusX];
U(:,:,7) = [PDM.RadiusY, -PDM.RadiusY];
U(:,:,8) = [-PDM.RadiusY, PDM.RadiusY];
U(:,:,9) = [PDM.Rx, PDM.Rx];           % Converge Rx
U(:,:,10) = [-PDM.Rx, -PDM.Rx];
U(:,:,11) = [PDM.Ry, PDM.Ry];          % Converge Ry
U(:,:,12) = [-PDM.Ry, -PDM.Ry];

val = [51 151 271 371]; 
kp_img = [ img( :, val ) ];

direction = 1;
best_FBTE = 10^200;
track_FBTE = [];
while direction > 0
    
         direction = 0;
    
         for i=1:10                                                              % Converge PDM
             
             data = data + U(:,:,i)*scale;
                                                
             kp = [ data( :, val ) ];                                           % Select keypoints

             T = mean(kp_img')' - mean(kp')';                                   % Compute [Tx, Ty]
             q = kp_img - repmat(T,1,4);

             ptsn = kp(:,1) / norm(kp(:,1));                                    % Compute Rz
             kp_imgn = q(:,1) / norm(q(:,1));

             Rz = atan2( kp_imgn(2,:), kp_imgn(1,:) ) - ...
                            atan2( ptsn(2,:), ptsn(1,:) ) 

             %kp_img_dist = mean( repmat( kp_img(:,1),1,4 )' - kp_img' )';
             %kp_dist = mean( repmat( kp(:,1),1,4 )' - kp' )';
             %S = [ kp_img_dist./kp_dist ];  
             
             M = [ cos(Rz) -sin(Rz) T(1); sin(Rz) ...                      % Compute M
                        cos(Rz) T(2); 0 0 1 ];

             FBTE = ForwardBackwardTransferError( M, kp, kp_img )               % Compute FBTE
         
             if best_FBTE > FBTE
                    best_FBTE = FBTE;
                    track_FBTE = [track_FBTE, FBTE];
                    direction = i;
             end
             
             data = data - U(:,:,i)*scale;
             
         end
         
         if direction ~= 0
            data = data + U(:,:,direction)*scale;
         end
end

Circle.FBTE = track_FBTE;
Circle.Contour = data;
Circle.M = M;
Circle.parameters.Rz = Rz;

pts = [data; ones(1,size(Template.pts,2))];                         % Render
pts = M*pts;
kp_draw = M*[kp; 1 1 1 1];
figure(3), scatter( pts(1,:), pts(2,:), 'r.')
figure(3), hold on, scatter( img(1,:), img(2,:), 'bx')
figure(3), scatter( kp_img(1,:), kp_img(2,:), 'g^')
figure(3), scatter( kp_draw(1,:), kp_draw(2,:), 'mo')
hold off, legend('Template', 'Target',  ... 
                    'kp: Template', 'kp: Target')

end

