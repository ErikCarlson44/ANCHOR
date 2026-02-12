%% ANCHOR — Mission Bay U-Net Obstacles (RED), Water-Only Nav, Zoomed/Panning Map
clear; close all; clc;

% ===================== USER KNOBS =====================
MAP_FILE        = 'mission-bay-map.jpg';
DO_TRAIN_UNET   = true;      % Uses U-Net if Deep Learning Toolbox is present; else HSV fallback
PATCH_SIZE      = [256 256];
EPOCHS          = 4;         % quick edge-refinement training

% World & camera
GS_BASE  = 10;               % legacy world
SCALE    = 100;              % 100× larger
GS       = GS_BASE*SCALE;    % 1000-world
ZOOM     = 3.0;              % on-screen zoom (2–4 is good)
CENTER   = [GS/2 GS/2];      % boat pinned here
OCC_ALPHA = 0.35;            % land overlay opacity

% APF (scale-aware)
zeta   = 1.1547;
eta    = 0.0732;
dstar  = 0.03*GS;
Qstar  = 0.075*GS;
ksink  = 1.6;
sig_sink = 0.06*GS;

% Motion
error_theta_max = deg2rad(45);
v_max    = 0.02*GS;
Kp_omega = 1.5;
omega_max = 0.5*pi;
dT       = 0.1;
t_max    = 9000;

% Shore/land safety
SHORE_PAD_PX     = 8;        % dilate land to avoid skimming shoreline
HARD_WATER_ONLY  = true;     % never allow pose outside water
% ======================================================

%% ============= Load map ===============================
I = imread(MAP_FILE);
[H, W, ~] = size(I);

%% ============= Auto HSV baseline mask =================
Ihsv = rgb2hsv(I);
Hh = Ihsv(:,:,1); Sh = Ihsv(:,:,2); Vh = Ihsv(:,:,3);
pre = (Sh>0.15) & (Vh>0.15) & (Hh>0.52) & (Hh<0.75);
if nnz(pre)>500
    mu = mean(Hh(pre),'all'); hw = 0.08;
    waterMask = (Hh>max(0,mu-hw)) & (Hh<min(1,mu+hw)) & (Sh>0.15) & (Vh>0.15);
else
    waterMask = pre;
end
waterMask = imclose(waterMask, strel('disk',3));
waterMask = imfill(waterMask,'holes');
waterMask = bwareaopen(waterMask, max(300, round(0.0001*H*W)));

%% ============= Optional U-Net refinement (disk datastores; no RandomSeed) ============
if DO_TRAIN_UNET && exist('unetLayers','file')==2
    % Write one image + label to a temp dataset (datastores expect files)
    rootTmp = fullfile(tempdir, 'mbay_unet_ds');
    imgDir  = fullfile(rootTmp, 'images');
    lblDir  = fullfile(rootTmp, 'labels');
    if ~exist(imgDir,'dir'), mkdir(imgDir); end
    if ~exist(lblDir,'dir'), mkdir(lblDir); end

    imwrite(I, fullfile(imgDir,'img.png'));

    % label: 1=water, 2=notwater
    L = uint8(ones(H,W)); L(~waterMask) = 2;
    imwrite(L, fullfile(lblDir,'label.png'));

    classNames = ["water","notwater"];
    labelIDs   = [1 2];

    imds = imageDatastore(imgDir);
    pxds = pixelLabelDatastore(lblDir, classNames, labelIDs);

    % Patch extraction — set RNG here; remove unsupported args
    rng(7);  % << replaces the old 'RandomSeed' name-value
    rpxds = randomPatchExtractionDatastore(imds, pxds, PATCH_SIZE, ...
        'PatchesPerImage', 600);   % keep only supported args

    lgraph = unetLayers([PATCH_SIZE 3], 2, 'EncoderDepth', 3);
    opts = trainingOptions('adam', 'InitialLearnRate',1e-3, ...
        'MaxEpochs',EPOCHS, 'MiniBatchSize',8, 'Shuffle','every-epoch', ...
        'Verbose',false,'Plots','none');

    net = trainNetwork(rpxds, lgraph, opts);

    seg = semanticseg(I, net, 'MiniBatchSize', 4);
    waterMask = seg == 'water';

    % Clean again
    waterMask = imclose(waterMask, strel('disk',3));
    waterMask = imfill(waterMask,'holes');
    waterMask = bwareaopen(waterMask, max(300, round(0.0001*H*W)));
elseif DO_TRAIN_UNET
    warning('Deep Learning Toolbox not found. Using HSV segmentation only.');
end

%% ============= Land obstacles & RED borders ============================
notWater = ~waterMask;
if SHORE_PAD_PX>0
    notWater = imdilate(notWater, strel('disk', SHORE_PAD_PX));
end

[B, ~] = bwboundaries(notWater,'noholes');
obst_all = [];
sampleEvery = 3;
for k = 1:numel(B)
    b = B{k};  if size(b,1) < 40, continue; end
    b = b(1:sampleEvery:end, :);
    if ~isequal(b(1,:), b(end,:)), b(end+1,:)=b(1,:); end
    px = b(:,2); py = b(:,1);
    xw = (px-1)/(W-1)*GS;
    yw = GS - (py-1)/(H-1)*GS;
    obst_all = [obst_all, [xw.'; yw.'], [NaN;NaN]]; %#ok<AGROW>
end

% Distance fields
D_land_px = bwdist(notWater);
[dDx_px, dDy_px] = gradient(D_land_px);
[D_to_water_px, idx_to_water] = bwdist(waterMask);

% Pixel <-> world helpers
px_from_x = @(x) (x/GS)*(W-1) + 1;
py_from_y = @(y) (1 - y/GS)*(H-1) + 1;
dpx_dx = (W-1)/GS;
dpy_dy = -(H-1)/GS;    % minus from y inversion
px_to_world_x = @(px) (px-1)/(W-1)*GS;
py_to_world_y = @(py) GS - (py-1)/(H-1)*GS;

%% ============= Start/Goal (ensure both in water) ======================
rng('shuffle');
x = GS*0.50; y = GS*0.50; theta = 0;

% if start not in water, snap to nearest water
ps = round(min(max(px_from_x(x),1),W));
qs = round(min(max(py_from_y(y),1),H));
if ~waterMask(qs,ps)
    idx = idx_to_water(qs,ps); [rw,cw]=ind2sub([H W],idx);
    x = px_to_world_x(cw); y = py_to_world_y(rw);
end

for tt=1:5000
    x_goal = 0.05*GS + 0.90*GS*rand;
    y_goal = 0.05*GS + 0.90*GS*rand;
    pxg = round(min(max(px_from_x(x_goal),1),W));
    pyg = round(min(max(py_from_y(y_goal),1),H));
    if waterMask(pyg,pxg), break; end
end
tol_goal = 0.005*GS;

%% ============= Visualization (zoomed, boat-centered, panning) =========
fig = figure('Name','Mission Bay — U-Net Obstacles (RED) & Water-Only Nav','NumberTitle','off');
clf; hold on; axis equal; box on; grid on;
xlim([0 GS]); ylim([0 GS]); set(gca,'YDir','normal');
xlabel('screen x'); ylabel('screen y');
title('Boat-centered (zoomed). Land bordered in RED. Water-only enforced.');

hMap = imagesc([0 GS],[0 GS], I); set(gca,'YDir','normal'); uistack(hMap,'bottom');
occImg = uint8(notWater)*255;
hOcc = imagesc([0 GS],[0 GS], occImg); set(gca,'YDir','normal'); colormap(gca,'gray'); alpha(hOcc,OCC_ALPHA);

hEdges = plot(NaN,NaN,'r-','LineWidth',1.6);
hGoal = plot(NaN,NaN,'ob','MarkerFaceColor','b','MarkerSize',6);
hPath = plot(NaN,NaN,'-b','LineWidth',1.6);
hRef  = plot(NaN,NaN,'-g','LineWidth',2);
hHead = plot(NaN,NaN,'-r','LineWidth',2);

recenterZ  = @(Px,Py,xb,yb) deal( ZOOM*(Px - xb) + CENTER(1), ZOOM*(Py - yb) + CENTER(2) );
shiftImgXY = @(xb,yb) deal( ZOOM*[0 GS] - ZOOM*xb + CENTER(1), ZOOM*[0 GS] - ZOOM*yb + CENTER(2) );

%% ============= Simulation ============================================
t=1; X=zeros(1,t_max); Y=zeros(1,t_max); X(1)=x; Y(1)=y;

while (norm([x y]-[x_goal y_goal]) > tol_goal) && (t < t_max)
    % Attractive + sink
    rvec = [x y] - [x_goal y_goal]; r = hypot(rvec(1),rvec(2));
    nablaU_att = (r <= dstar) * (zeta*rvec) + (r > dstar) * (zeta*dstar*rvec/max(r,1e-9));
    nablaU_sink = ksink * ([x y]-[x_goal y_goal]) * (1/sig_sink^2) * ...
                  exp(-((x-x_goal)^2+(y-y_goal)^2)/(2*sig_sink^2));

    % Land repulsion (pixels -> world)
    px = min(max(px_from_x(x),1),W);
    py = min(max(py_from_y(y),1),H);
    D_here_px  = interp2(D_land_px, px, py, 'linear', 0);
    dDx_here   = interp2(dDx_px,    px, py, 'linear', 0);
    dDy_here   = interp2(dDy_px,    px, py, 'linear', 0);
    dD_dx_world = dDx_here * dpx_dx;
    dD_dy_world = dDy_here * dpy_dy;
    scale_px_to_world = min(GS/(W-1), GS/(H-1));
    D_here_world = D_here_px * scale_px_to_world;
    if D_here_world <= Qstar && D_here_world > 1e-6
        coeff = eta*(1/Qstar - 1/D_here_world) * (1/D_here_world^2);
        nablaU_rep = coeff * [dD_dx_world, dD_dy_world];
    else
        nablaU_rep = [0 0];
    end

    % Total gradient
    nablaU = nablaU_att + nablaU_sink + nablaU_rep;

    % Heading/velocity
    theta_ref = atan2(-nablaU(2), -nablaU(1));
    err = mod(theta_ref - theta + pi, 2*pi) - pi;
    if abs(err) <= error_theta_max
        alpha = (error_theta_max - abs(err))/error_theta_max;
        v_ref = min(alpha*norm(-nablaU), v_max);
    else
        v_ref = 0;
    end
    omega_ref = min(max(Kp_omega*err, -omega_max), omega_max);

    % Predict next pose
    nx = x + v_ref*cos(theta)*dT;
    ny = y + v_ref*sin(theta)*dT;

    % ===== WATER-ONLY ENFORCEMENT =====
    pxn = round(min(max(px_from_x(nx),1),W));
    pyn = round(min(max(py_from_y(ny),1),H));
    if ~waterMask(pyn,pxn)
        % block entry and optionally snap to nearest water
        v_ref = 0;
        if HARD_WATER_ONLY
            idx = idx_to_water(pyn,pxn);
            [rw, cw] = ind2sub([H W], idx);
            x = px_to_world_x(cw);   y = py_to_world_y(rw);
        end
        theta = theta + omega_ref*dT;   % rotate only
    else
        theta = theta + omega_ref*dT;
        x = nx; y = ny;                 % safe advance
    end

    % Log & draw
    t = t+1; X(t)=x; Y(t)=y;

    [XD, YD] = shiftImgXY(x,y);
    set(hMap,'XData',XD,'YData',YD);
    set(hOcc,'XData',XD,'YData',YD);

    if ~isempty(obst_all)
        [ox,oy] = recenterZ(obst_all(1,:),obst_all(2,:), x,y);
        set(hEdges,'XData',ox,'YData',oy);
    end

    [gx_s,gy_s] = recenterZ(x_goal,y_goal,x,y); set(hGoal,'XData',gx_s,'YData',gy_s);
    [Xp, Yp]    = recenterZ(X(1:t),Y(1:t),x,y); set(hPath,'XData',Xp,'YData',Yp);
    set(hRef, 'XData',[CENTER(1) CENTER(1)+0.05*GS*cos(theta_ref)], ...
              'YData',[CENTER(2) CENTER(2)+0.05*GS*sin(theta_ref)]);
    set(hHead,'XData',[CENTER(1) CENTER(1)+0.05*GS*cos(theta)], ...
              'YData',[CENTER(2) CENTER(2)+0.05*GS*sin(theta)]);
    drawnow;
end

travel_time = (t-1)*dT;
disp("Travel time [s]: " + travel_time);
