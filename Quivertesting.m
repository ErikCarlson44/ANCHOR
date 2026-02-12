%% ANCHOR APF — Large Map (100x), Map+Grid Overlay, Boat-Centered Camera 
clear; close all; clc;

%% ---------------- World scaling ----------------
GS_BASE = 10;
SCALE   = 100;
GS      = GS_BASE * SCALE;        % 1000 x 1000 world units

%% ---------------- Try to read map ----------------
try
    imgMap = imread('mission-bay-map.jpg');
catch
    [gx, gy] = meshgrid(linspace(0,1,1024), linspace(0,1,1024));
    imgMap = uint8(255*cat(3, gx, gy, 0.5*ones(size(gx))));
    warning('Map image not found. Using a placeholder background.');
end

%% ---------------- Initial pose ----------------
x = GS/2;  y = GS/2;  theta = 0;

%% ---------------- Goal (random, avoid edges) ----------------
rng('shuffle');
margin = 0.02*GS;
tol_goal = 0.005*GS;
x_goal = margin + (GS - 2*margin)*rand;
y_goal = margin + (GS - 2*margin)*rand;

%% ---------------- APF parameters (scaled) ----------------
zeta   = 1.1547;          % attractive
eta    = 0.0732;          % repulsive
dstar  = 0.03*GS;
Qstar  = 0.075*GS;

% goal “sink”
ksink    = 1.8;
sig_sink = 0.06*GS;

% kinematics
error_theta_max = deg2rad(45);
v_max    = 0.02*GS;
Kp_omega = 1.5;
omega_max = 0.5*pi;

%% ---------------- Obstacles (CLOSED) scaled to GS ----------------
N = 100;  r_clear=0.01*GS;  wedge_deg=25;  R_keepout=GS;
theta_goal = atan2(y_goal - y, x_goal - x);
wedge_rad  = deg2rad(wedge_deg);

function_in_wedge = @(P) any( (abs(mod(atan2(P(2,:)-y, P(1,:)-x) - theta_goal + pi, 2*pi) - pi) <= wedge_rad) ...
                             & (sqrt((P(1,:)-x).^2 + (P(2,:)-y).^2) <= R_keepout) );
function_all_in_bounds = @(P) all(P(1,:)>=0 & P(1,:)<=GS & P(2,:)>=0 & P(2,:)<=GS);
function_clear_ok = @(P,isClosed) ( (~isClosed || ~inpolygon(x,y,P(1,:),P(2,:))) ...
                                 && min(hypot(P(1,:)-x, P(2,:)-y)) > r_clear );

% obst1: rotated rectangle
tries=0; while true
    tries=tries+1; if tries>400, error('Could not place obst1'); end
    cx=0.10*GS + 0.80*GS*rand; cy=0.10*GS + 0.80*GS*rand;
    w = GS*(0.08 + 0.16*rand); h = GS*(0.08 + 0.16*rand); th = 2*pi*rand;
    C=[-w/2 -h/2; w/2 -h/2; w/2 h/2; -w/2 h/2]';
    Rz=[cos(th) -sin(th); sin(th) cos(th)]; V=Rz*C+[cx;cy];
    e1=[linspace(V(1,1),V(1,2),N); linspace(V(2,1),V(2,2),N)];
    e2=[linspace(V(1,2),V(1,3),N); linspace(V(2,2),V(2,3),N)];
    e3=[linspace(V(1,3),V(1,4),N); linspace(V(2,3),V(2,4),N)];
    e4=[linspace(V(1,4),V(1,1),N); linspace(V(2,4),V(2,1),N)];
    P=[e1 e2 e3 e4];
    if ~function_all_in_bounds(P), continue; end
    if function_in_wedge(P), continue; end
    if ~function_clear_ok(P,true), continue; end
    obst1_points=P; break;
end

% obst2: annular “C”
tries=0; while true
    tries=tries+1; if tries>600, error('Could not place obst2'); end
    cx=0.15*GS + 0.70*GS*rand; cy=0.15*GS + 0.70*GS*rand;
    R2=GS*(0.09 + 0.15*rand); tck=GS*(0.035 + 0.045*rand); R1=max(0.03*GS,R2-tck);
    phi=(randi(4)-1)*pi/2; N_arc=N; N_rad=max(20,round(N*0.3));
    t=linspace(0,pi/2,N_arc);
    Ox=R2*cos(t); Oy=R2*sin(t); Ix=R1*cos(fliplr(t)); Iy=R1*sin(fliplr(t));
    c1x=linspace(R2*cos(pi/2),R1*cos(pi/2),N_rad); c1y=linspace(R2*sin(pi/2),R1*sin(pi/2),N_rad);
    c0x=linspace(R1,R2,N_rad); c0y=zeros(1,N_rad);
    Px=[Ox c1x Ix c0x]; Py=[Oy c1y Iy c0y];
    Rz=[cos(phi) -sin(phi); sin(phi) cos(phi)]; P2=Rz*[Px;Py]+[cx;cy];
    if ~function_all_in_bounds(P2), continue; end
    if function_in_wedge(P2), continue; end
    if ~function_clear_ok(P2,true), continue; end
    obst2_points=P2; break;
end

% obst3: rotated rectangle
tries=0; while true
    tries=tries+1; if tries>400, error('Could not place obst3'); end
    cx=0.10*GS + 0.80*GS*rand; cy=0.10*GS + 0.80*GS*rand;
    w=GS*(0.09+0.17*rand); h=GS*(0.09+0.17*rand); th=2*pi*rand;
    C=[-w/2 -h/2; w/2 -h/2; w/2 h/2; -w/2 h/2]'; Rz=[cos(th) -sin(th); sin(th) cos(th)];
    V=Rz*C+[cx;cy];
    e1=[linspace(V(1,1),V(1,2),N); linspace(V(2,1),V(2,2),N)];
    e2=[linspace(V(1,2),V(1,3),N); linspace(V(2,2),V(2,3),N)];
    e3=[linspace(V(1,3),V(1,4),N); linspace(V(2,3),V(2,4),N)];
    e4=[linspace(V(1,4),V(1,1),N); linspace(V(2,4),V(2,1),N)];
    P3=[e1 e2 e3 e4];
    if ~function_all_in_bounds(P3), continue; end
    if function_in_wedge(P3), continue; end
    if ~function_clear_ok(P3,true), continue; end
    obst3_points=P3; break;
end

% border polyline (inset by m)
Nb=800; m=0.01*GS;
border_points=[ linspace(m, GS-m, Nb)   linspace(GS-m, GS-m, Nb)  linspace(GS-m, m, Nb)   linspace(m, m, Nb) ;
                linspace(m, m, Nb)     linspace(m, GS-m, Nb)     linspace(GS-m, GS-m, Nb) linspace(GS-m, m, Nb) ];

%% ---------------- CUT-GRID occupancy ----------------
Nx=220; Ny=220;
x_edges = linspace(0,GS,Nx+1);  y_edges = linspace(0,GS,Ny+1);
xc = 0.5*(x_edges(1:end-1)+x_edges(2:end));
yc = 0.5*(y_edges(1:end-1)+y_edges(2:end));
[XC,YC] = meshgrid(xc,yc);
W = zeros(Ny,Nx);

S = 3; u = linspace(0,1,S);
for iy=1:Ny
    y0=y_edges(iy); y1=y_edges(iy+1);
    for ix=1:Nx
        x0=x_edges(ix); x1=x_edges(ix+1);
        [SX,SY]=meshgrid(x0+(x1-x0)*u, y0+(y1-y0)*u);
        inside = false(size(SX));
        inside = inside | inpolygon(SX,SY,obst1_points(1,:),obst1_points(2,:));
        inside = inside | inpolygon(SX,SY,obst2_points(1,:),obst2_points(2,:));
        inside = inside | inpolygon(SX,SY,obst3_points(1,:),obst3_points(2,:));
        W(iy,ix) = mean(inside(:));
    end
end

gauss1d = @(s) exp(-((-ceil(3*s):ceil(3*s)).^2)/(2*s^2));
s1=1.2; g1=gauss1d(s1); K = (g1'*g1)/sum(g1)^2;
W_smooth = conv2(W, K, 'same');

[Wgy, Wgx] = gradient(W_smooth, mean(diff(yc)), mean(diff(xc)));
kg = 1.25;

%% ---------------- Visualization (boat-centered) ----------------
CENTER = [GS/2 GS/2];

fig = figure('Name','Map + Grid Overlay — Boat Centered','NumberTitle','off');
clf; hold on; axis equal; box on; grid on;
xlim([0 GS]); ylim([0 GS]); set(gca,'YDir','normal');
xlabel('screen x'); ylabel('screen y');
title('ANCHOR APF — Large Map with Occupancy Grid Overlay');

hMap = imagesc([0 GS],[0 GS], imgMap); set(gca,'YDir','normal'); uistack(hMap,'bottom');
hOcc = imagesc([0 GS],[0 GS], 1-W);     set(gca,'YDir','normal'); colormap(gca,'gray'); alpha(hOcc,0.45);

hOb1 = plot(obst1_points(1,:), obst1_points(2,:), 'r-', 'LineWidth', 1.5);
hOb2 = plot(obst2_points(1,:), obst2_points(2,:), 'r-', 'LineWidth', 1.5);
hOb3 = plot(obst3_points(1,:), obst3_points(2,:), 'r-', 'LineWidth', 1.5);
hBrd = plot(border_points(1,:), border_points(2,:), 'r-', 'LineWidth', 1.5);

hGoal = plot(NaN,NaN,'ob','MarkerFaceColor','b');
hPath = plot(NaN,NaN,'-b','LineWidth',1.5);
hRef  = plot(NaN,NaN,'-g','LineWidth',2);
hHead = plot(NaN,NaN,'-r','LineWidth',2);

recenter = @(Px,Py,xb,yb) deal(Px - xb + CENTER(1), Py - yb + CENTER(2));

%% ===== ADD: waypoint vectors (goal vector, total APF vector) =====
% Waypoint/APF vectors drawn at the boat (CENTER)
hF_goal = quiver(CENTER(1),CENTER(2),0,0, ...
    'Color',[0 0.4 1],'LineWidth',2,'MaxHeadSize',2,'AutoScale','off');  % blue

hF_tot  = quiver(CENTER(1),CENTER(2),0,0, ...
    'Color',[1 0 0],'LineWidth',2,'MaxHeadSize',2,'AutoScale','off');     % red

legend([hF_goal hF_tot],{'F_{goal}','F_{total}'},'Location','southoutside');

%% ===== ADD: “waypoint finder” (look-ahead point along -∇U) =====
LOOKAHEAD_STEPS = 25;   % short preview
LOOKAHEAD_STEP  = 0.01*GS;
hWP    = plot(NaN,NaN,'mo','MarkerFaceColor','m','MarkerSize',6);   % next waypoint marker
hWPath = plot(NaN,NaN,'m--','LineWidth',1.2);                       % preview polyline

%% ===== ADD: 3-D potential surface (peaks & valleys) =====
%% ===== 3-D potential surface (individual peaks & sink) =====
SHOW_3D = true; POT_STRIDE = 8;
if SHOW_3D
    gx = linspace(0,GS,161); gy = linspace(0,GS,161);
    [GX,GY] = meshgrid(gx,gy);

    % --- Base APF terms (same math as before) ---
    R = hypot(GX - x_goal, GY - y_goal);
    U_att = zeros(size(GX));
    mask_in  = R <= dstar; mask_out = ~mask_in;
    U_att(mask_in)  = 0.5 * zeta * (R(mask_in).^2);
    U_att(mask_out) = dstar*zeta*R(mask_out) - 0.5*zeta*dstar^2;

    Q = [GX(:) GY(:)];
    [~, d1] = dsearchn(obst1_points', Q);
    [~, d2] = dsearchn(obst2_points', Q);
    [~, d3] = dsearchn(obst3_points', Q);
    [~, db] = dsearchn(border_points', Q);

    repf = @(dd) (0.5*eta*((1./max(dd,1e-6) - 1/Qstar).^2)) .* (dd <= Qstar);
    U_rep = repf(d1) + repf(d2) + repf(d3) + repf(db);
    U_rep = reshape(U_rep, size(GX));

    % grid “mountain” from cut-grid (low amplitude background)
    Wn = (W_smooth - min(W_smooth(:))) / max(eps, (max(W_smooth(:))-min(W_smooth(:))));
    U_grid = kg * imresize(Wn, size(GX), 'bilinear');

    U_base = U_att + U_rep + U_grid;

    % --- ADD: explicit Gaussian peaks for each obstacle + Gaussian sink at goal ---
    ps1 = polyshape(obst1_points(1,:),obst1_points(2,:)); [c1x,c1y] = centroid(ps1);
    ps2 = polyshape(obst2_points(1,:),obst2_points(2,:)); [c2x,c2y] = centroid(ps2);
    ps3 = polyshape(obst3_points(1,:),obst3_points(2,:)); [c3x,c3y] = centroid(ps3);

    G2d = @(x0,y0,s) exp(-((GX-x0).^2 + (GY-y0).^2)/(2*s^2));

    Aobs   = 0.40*max(U_base(:));   % obstacle peak height
    sigObs = 0.05*GS;               % obstacle peak width
    U_peaks = Aobs * ( G2d(c1x,c1y,sigObs) + G2d(c2x,c2y,sigObs) + G2d(c3x,c3y,sigObs) );

    Agoal   = 0.80*max(U_base(:));  % goal sink depth
    sigGoal = 0.06*GS;              % goal sink width
    U_goalSink = -Agoal * G2d(x_goal,y_goal,sigGoal);

    % final surface used for visualization
    U_vis = U_base + U_peaks + U_goalSink;

    % --- Plot surface + markers ---
    fig3 = figure('Name','3D Potential Field (peaks & sink)','NumberTitle','off'); clf;
    s = surf(GX,GY,U_vis,'EdgeColor','none'); hold on; box on; grid on;
    colormap(jet); colorbar; view(45,30);
    title('Total Potential U(x,y): obstacle peaks + goal sink'); xlabel('x'); ylabel('y'); zlabel('U');

    Ztop = max(U_vis(:))*1.02;
    plot3(obst1_points(1,:),obst1_points(2,:),Ztop*ones(1,size(obst1_points,2)),'r-','LineWidth',1.4);
    plot3(obst2_points(1,:),obst2_points(2,:),Ztop*ones(1,size(obst2_points,2)),'r-','LineWidth',1.4);
    plot3(obst3_points(1,:),obst3_points(2,:),Ztop*ones(1,size(obst3_points,2)),'r-','LineWidth',1.4);
    plot3(border_points(1,:),border_points(2,:),Ztop*ones(1,size(border_points,2)),'r-','LineWidth',1.2);

    % peak markers at obstacle centroids
    Zo1 = interp2(GX,GY,U_vis,c1x,c1y);  Zo2 = interp2(GX,GY,U_vis,c2x,c2y);  Zo3 = interp2(GX,GY,U_vis,c3x,c3y);
    scatter3([c1x c2x c3x],[c1y c2y c3y],[Zo1 Zo2 Zo3],50,[1 0 0],'filled');  % red peaks

    % sink marker at goal
    Zg  = interp2(GX,GY,U_vis,x_goal,y_goal);
    scatter3(x_goal,y_goal,Zg,70,[0 0.4 1],'filled');  % blue sink
    text(x_goal,y_goal,Zg,'  waypoint','Color',[0 0.4 1],'FontWeight','bold');

    % live path on this surface
    Z0 = interp2(GX,GY,U_vis, x, y, 'linear', Ztop);
    hPath3 = plot3(x, y, Z0, 'b-', 'LineWidth',2);
end

%% ---------------- Simulation loop ----------------
dT = 0.1; t = 1; t_max = 8000;
X = zeros(1,t_max); Y = zeros(1,t_max); X(1)=x; Y(1)=y;

while (norm([x y]-[x_goal y_goal]) > tol_goal) && (t < t_max)

    %% Attractive gradient (Khatib)
    rvec = [x y]-[x_goal y_goal]; r = hypot(rvec(1),rvec(2));
    if r <= dstar, nablaU_att = zeta * rvec;
    else,          nablaU_att = zeta * dstar * rvec / max(r,1e-9);
    end

    %% Repulsive gradients (nearest vertices approx)
    nablaU_rep = [0 0]; grad1=[0 0]; grad2=[0 0]; grad3=[0 0];
    [i1,d1p] = dsearchn(obst1_points', [x y]);
    [i2,d2p] = dsearchn(obst2_points', [x y]);
    [i3,d3p] = dsearchn(obst3_points', [x y]);
    [ib,dbp] = dsearchn(border_points', [x y]);
    if d1p <= Qstar
        p = obst1_points(:,i1)'; v = ([x y]-p);
        grad1 = (eta*(1/Qstar - 1/d1p) * 1/max(d1p^2,1e-9))*v; nablaU_rep = nablaU_rep + grad1;
    end
    if (d2p <= Qstar) && ~inpolygon(x,y,obst2_points(1,:),obst2_points(2,:))
        p = obst2_points(:,i2)'; v = ([x y]-p);
        grad2 = (eta*(1/Qstar - 1/d2p) * 1/max(d2p^2,1e-9))*v; nablaU_rep = nablaU_rep + grad2;
    end
    if (d3p <= Qstar) && ~inpolygon(x,y,obst3_points(1,:),obst3_points(2,:))
        p = obst3_points(:,i3)'; v = ([x y]-p);
        grad3 = (eta*(1/Qstar - 1/d3p) * 1/max(d3p^2,1e-9))*v; nablaU_rep = nablaU_rep + grad3;
    end
    if dbp <= Qstar
        p = border_points(:,ib)'; v = ([x y]-p);
        nablaU_rep = nablaU_rep + (eta*(1/Qstar - 1/dbp) * 1/max(dbp^2,1e-9))*v;
    end

    %% Occupancy mountain gradient
    gWx = interp2(XC,YC, Wgx, x, y, 'linear', 0);
    gWy = interp2(XC,YC, Wgy, x, y, 'linear', 0);
    nablaU_grid = kg * [gWx, gWy];

    %% Goal sink gradient
    nablaU_sink = ksink * ([x y]-[x_goal y_goal]) * (1/sig_sink^2) * ...
                  exp(-((x-x_goal)^2+(y-y_goal)^2)/(2*sig_sink^2));

    %% Total gradient (note: forces are -∇U)
    nablaU = nablaU_att + nablaU_rep + nablaU_grid + nablaU_sink;
    F_goal  = -(nablaU_att + nablaU_sink);
    F_total = -nablaU;

    %% Heading/velocity and integrate
    theta_ref  = atan2(F_total(2), F_total(1));        % follow total force
    error_theta = mod(theta_ref - theta + pi, 2*pi) - pi;
    if abs(error_theta) <= error_theta_max
        alpha = (error_theta_max - abs(error_theta))/error_theta_max;
        v_ref = min(alpha*norm(F_total), v_max);
    else
        v_ref = 0;
    end
    omega_ref = min(max(Kp_omega*error_theta, -omega_max), omega_max);
    theta = theta + omega_ref*dT;
    x = x + v_ref*cos(theta)*dT;
    y = y + v_ref*sin(theta)*dT;

    t = t+1; X(t)=x; Y(t)=y;

    %% ------------------- DRAW (boat centered) -------------------
    set(hMap,'XData', [0 GS]-x + CENTER(1), 'YData', [0 GS]-y + CENTER(2));
    set(hOcc,'XData', [0 GS]-x + CENTER(1), 'YData', [0 GS]-y + CENTER(2));

    [o1x,o1y] = recenter(obst1_points(1,:),obst1_points(2,:),x,y); set(hOb1,'XData',o1x,'YData',o1y);
    [o2x,o2y] = recenter(obst2_points(1,:),obst2_points(2,:),x,y); set(hOb2,'XData',o2x,'YData',o2y);
    [o3x,o3y] = recenter(obst3_points(1,:),obst3_points(2,:),x,y); set(hOb3,'XData',o3x,'YData',o3y);
    [bx,by]   = recenter(border_points(1,:),border_points(2,:),x,y); set(hBrd,'XData',bx,'YData',by);

    [gx_s,gy_s] = recenter(x_goal,y_goal,x,y); set(hGoal,'XData',gx_s,'YData',gy_s);
    [Xp, Yp]    = recenter(X(1:t),Y(1:t),x,y); set(hPath,'XData',Xp,'YData',Yp);
    set(hRef, 'XData',[CENTER(1) CENTER(1)+0.05*GS*cos(theta_ref)], ...
              'YData',[CENTER(2) CENTER(2)+0.05*GS*sin(theta_ref)]);
    set(hHead,'XData',[CENTER(1) CENTER(1)+0.05*GS*cos(theta)], ...
              'YData',[CENTER(2) CENTER(2)+0.05*GS*sin(theta)]);
    

    % ---- UPDATE waypoint vectors (drawn at the boat which is at CENTER) ----
    sf = 0.15*GS;   % just for drawing scale
    Ft = F_total / max(norm(F_total),1e-9);
    Fg = F_goal  / max(norm(F_goal),1e-9);
    set(hF_tot, 'XData',CENTER(1),'YData',CENTER(2),'UData',sf*Ft(1),'VData',sf*Ft(2));
set(hF_goal,'XData',CENTER(1),'YData',CENTER(2),'UData',sf*Fg(1),'VData',sf*Fg(2));

    % ---- LOOK-AHEAD waypoint preview along -∇U (greedy “finder”) ----
    xa = x; ya = y;
    XpLW = zeros(1,LOOKAHEAD_STEPS); YpLW = zeros(1,LOOKAHEAD_STEPS);
    for kk=1:LOOKAHEAD_STEPS
        % recompute grad pieces at (xa,ya) cheaply
        rvec = [xa ya]-[x_goal y_goal]; r = hypot(rvec(1),rvec(2));
        if r <= dstar, g_att = zeta*rvec; else, g_att = zeta*dstar*rvec/max(r,1e-9); end
        g_sink = ksink * ([xa ya]-[x_goal y_goal]) * (1/sig_sink^2) * ...
                 exp(-((xa-x_goal)^2+(ya-y_goal)^2)/(2*sig_sink^2));
        g_rep = [0 0];
        [i1,d1p] = dsearchn(obst1_points', [xa ya]);
        if d1p<=Qstar, p=obst1_points(:,i1)'; g_rep=g_rep+(eta*(1/Qstar-1/d1p)/max(d1p^2,1e-9))*([xa ya]-p); end
        [i2,d2p] = dsearchn(obst2_points', [xa ya]);
        if d2p<=Qstar && ~inpolygon(xa,ya,obst2_points(1,:),obst2_points(2,:)), p=obst2_points(:,i2)'; g_rep=g_rep+(eta*(1/Qstar-1/d2p)/max(d2p^2,1e-9))*([xa ya]-p); end
        [i3,d3p] = dsearchn(obst3_points', [xa ya]);
        if d3p<=Qstar && ~inpolygon(xa,ya,obst3_points(1,:),obst3_points(2,:)), p=obst3_points(:,i3)'; g_rep=g_rep+(eta*(1/Qstar-1/d3p)/max(d3p^2,1e-9))*([xa ya]-p); end
        [ib,dbp] = dsearchn(border_points', [xa ya]);
        if dbp<=Qstar, p=border_points(:,ib)'; g_rep=g_rep+(eta*(1/Qstar-1/dbp)/max(dbp^2,1e-9))*([xa ya]-p); end
        gWx = interp2(XC,YC, Wgx, xa, ya, 'linear', 0);
        gWy = interp2(XC,YC, Wgy, xa, ya, 'linear', 0);
        g_grid = kg*[gWx,gWy];

        g_tot = g_att + g_sink + g_rep + g_grid;
        step = LOOKAHEAD_STEP * g_tot / max(norm(g_tot), 1e-9);
        xa = xa - step(1); ya = ya - step(2);   % descent step
        XpLW(kk)=xa; YpLW(kk)=ya;
    end
    [wx,wy] = recenter(xa,ya,x,y);        % final look-ahead point
    set(hWP,'XData',wx,'YData',wy);
    [wpx,wpy] = recenter(XpLW,YpLW,x,y);  % preview line
    set(hWPath,'XData',wpx,'YData',wpy);

    drawnow;

    %% ----- 3-D: extend path on potential surface -----
    if SHOW_3D && mod(t,POT_STRIDE)==0
        Zt = interp2(GX,GY,U_vis, x, y, 'linear', max(U_vis(:)));
        set(hPath3,'XData',[get(hPath3,'XData') x], ...
                   'YData',[get(hPath3,'YData') y], ...
                   'ZData',[get(hPath3,'ZData') Zt]);
        drawnow limitrate;
    end
end

travel_time = (t-1)*dT;
disp("Travel time [s]: " + travel_time);
