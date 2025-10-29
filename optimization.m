%% ANCHOR APF — Large Map (100x) / Many Obstacles / Boat-Centered / 3D Peaks&Sink
clear; close all; clc;

%% ---------------- World scaling ----------------
GS_BASE = 10; SCALE = 100; GS = GS_BASE * SCALE;   % 1000 x 1000 world

%% ---------------- Map (background) ----------------
try
    imgMap = imread('mission-bay-map.jpg');
catch
    [gx, gy] = meshgrid(linspace(0,1,1024), linspace(0,1,1024));
    imgMap = uint8(255*cat(3, gx, gy, 0.5*ones(size(gx))));
    warning('Map image not found. Using a placeholder background.');
end

%% ---------------- Initial pose ----------------
rng('shuffle');
x = GS/2; y = GS/2; theta = 0;

%% ---------------- Random goal (initial pick) ----------------
margin = 0.02*GS;                    % 2% edges margin
tol_goal = 0.005*GS;                 % stop tolerance
x_goal = margin + (GS - 2*margin)*rand;
y_goal = margin + (GS - 2*margin)*rand;

%% ---------------- APF parameters (scaled) ----------------
zeta = 1.1547;              % attractive gain
eta  = 0.1200;              % stronger repulsion to force detours
dstar = 0.03*GS;
Qstar = 0.085*GS;           % thicker influence band

% goal “sink”
ksink    = 1.8;
sig_sink = 0.06*GS;

% kinematics
error_theta_max = deg2rad(45);
v_max    = 0.020*GS;
Kp_omega = 1.7;
omega_max = 0.5*pi;

%% ---------------- Helper lambdas (no nested functions) ----------------
in_wedge = @(P,th0,wedger,xc,yc) any( (abs(mod(atan2(P(2,:)-yc, P(1,:)-xc) - th0 + pi, 2*pi) - pi) <= wedger) );
all_in   = @(P) all(P(1,:)>=0 & P(1,:)<=GS & P(2,:)>=0 & P(2,:)<=GS);  % <-- 1-arg closure
clear_ok = @(P,isClosed,xc,yc,rclr) ( (~isClosed || ~inpolygon(xc,yc,P(1,:),P(2,:))) ...
                                   && min(hypot(P(1,:)-xc, P(2,:)-yc)) > rclr );

Rot = @(th)[cos(th) -sin(th); sin(th) cos(th)];

% Axis-aligned rectangle polyline, then rotate + translate: returns 2xM
poly_rect = @(cx,cy,w,h,th,N) ...
    ( Rot(th) * [ linspace(-w/2,  w/2, N),   w/2*ones(1,N),          linspace( w/2, -w/2, N),  -w/2*ones(1,N);
                  -h/2*ones(1,N),           linspace(-h/2, h/2, N),   h/2*ones(1,N),            linspace( h/2, -h/2, N) ] ...
      + [cx;cy] * ones(1, 4*N) );

% Circle polyline: returns 2xN
poly_circle = @(cx,cy,R,N) [cx + R*cos(linspace(0,2*pi,N)); cy + R*sin(linspace(0,2*pi,N))];

% “C” annular sector: build in local frame, then rotate + translate
poly_csector = @(cx,cy,R1,R2,phi,N) ...
    ( Rot(phi) * [ R2*cos(linspace(0,pi/2,N)),                                  linspace(R2*cos(pi/2),R1*cos(pi/2),round(0.3*N)),  R1*cos(linspace(pi/2,0,N)),                 linspace(R1, R2, round(0.3*N));
                   R2*sin(linspace(0,pi/2,N)),                                  linspace(R2*sin(pi/2),R1*sin(pi/2),round(0.3*N)),  R1*sin(linspace(pi/2,0,N)),                 zeros(1,round(0.3*N)) ] ...
      + [cx;cy] * ones(1, N + round(0.3*N) + N + round(0.3*N)) );

% Repulsive potential (closure captures Qstar, eta)
repf = @(dd) (0.5*eta*((1./max(dd,1e-6) - 1/Qstar).^2)) .* (dd <= Qstar);  % <-- closure

%% ---------------- Build MANY obstacles ----------------
Nedge=80; r_clear=0.01*GS; wedge_deg=25; R_keepout=GS;
theta_goal0 = atan2(y_goal - y, x_goal - x); wedge_rad = deg2rad(wedge_deg);

obs = {};  % cell array of 2xM polylines (CLOSED)

% a) rectangles
for k=1:5
    tries=0;
    while true
        tries=tries+1; if tries>600, break; end
        cx=0.08*GS + 0.84*GS*rand; cy=0.08*GS + 0.84*GS*rand;
        w = GS*(0.05 + 0.12*rand); h = GS*(0.05 + 0.14*rand); th = 2*pi*rand;
        P = poly_rect(cx,cy,w,h,th,Nedge);
        if ~all_in(P), continue; end
        if in_wedge(P,theta_goal0,wedge_rad,x,y), continue; end
        if ~clear_ok(P,true,x,y,r_clear), continue; end
        obs{end+1} = P; break;
    end
end
% b) circular “islands”
for k=1:4
    tries=0;
    while true
        tries=tries+1; if tries>600, break; end
        cx=0.10*GS + 0.80*GS*rand; cy=0.10*GS + 0.80*GS*rand;
        R = GS*(0.04 + 0.06*rand);
        P = poly_circle(cx,cy,R,Nedge);
        if ~all_in(P), continue; end
        if in_wedge(P,theta_goal0,wedge_rad,x,y), continue; end
        if ~clear_ok(P,true,x,y,r_clear), continue; end
        obs{end+1} = P; break;
    end
end
% c) C-sectors (channels)
for k=1:3
    tries=0;
    while true
        tries=tries+1; if tries>600, break; end
        cx=0.12*GS + 0.76*GS*rand; cy=0.12*GS + 0.76*GS*rand;
        R2 = GS*(0.09 + 0.15*rand); tck = GS*(0.035 + 0.055*rand); R1 = max(0.03*GS,R2-tck);
        phi=(randi(4)-1)*pi/2;
        P = poly_csector(cx,cy,R1,R2,phi,Nedge);
        if ~all_in(P), continue; end
        if in_wedge(P,theta_goal0,wedge_rad,x,y), continue; end
        if ~clear_ok(P,true,x,y,r_clear), continue; end
        obs{end+1} = P; break;
    end
end

% border (no-go)
Nb=1000; m=0.01*GS;
border_points=[ linspace(m, GS-m, Nb)   linspace(GS-m, GS-m, Nb)  linspace(GS-m, m, Nb)   linspace(m, m, Nb) ;
                linspace(m, m, Nb)     linspace(m, GS-m, Nb)     linspace(GS-m, GS-m, Nb) linspace(GS-m, m, Nb) ];

%% ---------------- Ensure the GOAL is valid ----------------
% Not inside obstacles and at least 2% GS away from any obstacle/border
min_goal_clear = 0.01*GS;
tries=0;
while true
    tries=tries+1; if tries>2000, error('Could not place a valid goal'); end
    inside=false;
    for j=1:numel(obs)
        if inpolygon(x_goal,y_goal,obs{j}(1,:),obs{j}(2,:)), inside=true; break; end
    end
    if inside
        x_goal = margin + (GS - 2*margin)*rand;
        y_goal = margin + (GS - 2*margin)*rand;
        continue;
    end
    dmin = inf;
    for j=1:numel(obs)
        [~, dj] = dsearchn(obs{j}', [x_goal y_goal]); dmin = min(dmin, dj);
    end
    [~, db] = dsearchn(border_points', [x_goal y_goal]); dmin = min(dmin, db);
    if dmin >= min_goal_clear, break; end
    x_goal = margin + (GS - 2*margin)*rand;
    y_goal = margin + (GS - 2*margin)*rand;
end

%% ---------------- CUT-GRID occupancy (for black overlay + “mountain”) ----------------
Nx=240; Ny=240;
x_edges = linspace(0,GS,Nx+1); y_edges = linspace(0,GS,Ny+1);
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
        for j=1:numel(obs)
            P = obs{j};
            inside = inside | inpolygon(SX,SY,P(1,:),P(2,:));
        end
        W(iy,ix) = mean(inside(:));
    end
end

% smooth occupancy -> continuous ridge
gauss1d = @(s) exp(-((-ceil(3*s):ceil(3*s)).^2)/(2*s^2));
s1=1.2; g1=gauss1d(s1); K = (g1'*g1)/sum(g1)^2;
W_smooth = conv2(W, K, 'same');
[Wgy, Wgx] = gradient(W_smooth, mean(diff(yc)), mean(diff(xc)));
kg = 1.25;                     % grid mountain scale

%% ---------------- Visualization: boat-centered top-down ----------------
CENTER = [GS/2 GS/2];

fig = figure('Name','Map + Grid Overlay — Boat Centered','NumberTitle','off');
clf; hold on; axis equal; box on; grid on; set(gca,'YDir','normal');
xlim([0 GS]); ylim([0 GS]);
xlabel('screen x'); ylabel('screen y');
title('ANCHOR APF — Many Obstacles with Occupancy Grid Overlay');

hMap = imagesc([0 GS],[0 GS], imgMap); set(gca,'YDir','normal'); uistack(hMap,'bottom');
hOcc = imagesc([0 GS],[0 GS], 1-W);   set(gca,'YDir','normal'); colormap(gca,'gray'); alpha(hOcc,0.45);

% draw obstacles
hObs = gobjects(numel(obs),1);
for j=1:numel(obs)
    P = obs{j};
    hObs(j) = plot(P(1,:),P(2,:),'r-','LineWidth',1.6);
end
hBrd = plot(border_points(1,:),border_points(2,:),'r-','LineWidth',1.6);

% boat trail & headings
hGoal = plot(NaN,NaN,'ob','MarkerFaceColor','b');
hPath = plot(NaN,NaN,'-b','LineWidth',1.5);
hRef  = plot(NaN,NaN,'-g','LineWidth',2);
hHead = plot(NaN,NaN,'-r','LineWidth',2);

% force arrows at boat (goal + total)
hF_goal = quiver(CENTER(1),CENTER(2),0,0,'Color',[0 0.4 1],'LineWidth',2,'AutoScale','off','MaxHeadSize',2);
hF_tot  = quiver(CENTER(1),CENTER(2),0,0,'Color',[1 0 0],  'LineWidth',2,'AutoScale','off','MaxHeadSize',2);

% look-ahead waypoint preview
LOOKAHEAD_STEPS = 25; LOOKAHEAD_STEP = 0.010*GS;
hWP    = plot(NaN,NaN,'mo','MarkerFaceColor','m','MarkerSize',6);
hWPath = plot(NaN,NaN,'m--','LineWidth',1.2);

recenter = @(Px,Py,xb,yb) deal(Px - xb + CENTER(1), Py - yb + CENTER(2));

%% ---------------- 3-D potential with distinct peaks & goal sink ----------------
SHOW_3D = true; POT_STRIDE = 8;
if SHOW_3D
    gx = linspace(0,GS,161); gy = linspace(0,GS,161);
    [GX,GY] = meshgrid(gx,gy);

    % Attractive
    R = hypot(GX - x_goal, GY - y_goal);
    U_att = zeros(size(GX));
    inR = R<=dstar; U_att(inR)=0.5*zeta*(R(inR).^2);
    U_att(~inR) = dstar*zeta*R(~inR) - 0.5*zeta*dstar^2;

    % Repulsive (sum over all obstacles + border)
    Q = [GX(:) GY(:)];
    U_rep = zeros(numel(GX),1);
    for j=1:numel(obs)
        [~, dj] = dsearchn(obs{j}', Q);
        U_rep = U_rep + repf(dj);        % <-- closure usage
    end
    [~, db] = dsearchn(border_points', Q);
    U_rep = U_rep + repf(db);            % <-- closure usage
    U_rep = reshape(U_rep,size(GX));

    % grid mountain (low amp)
    Wn = (W_smooth - min(W_smooth(:))) / max(eps, (max(W_smooth(:))-min(W_smooth(:))));
    U_grid = kg * imresize(Wn, size(GX), 'bilinear');

    U_base = U_att + U_rep + U_grid;

    % explicit Gaussian peaks at obstacle centroids + goal sink
    G2d = @(x0,y0,s) exp(-((GX-x0).^2 + (GY-y0).^2)/(2*s^2));
    Aobs   = 0.40*max(U_base(:));
    sigObs = 0.05*GS;
    U_peaks = zeros(size(GX));
    centroids = zeros(numel(obs),2);
    for j=1:numel(obs)
        ps = polyshape(obs{j}(1,:),obs{j}(2,:)); [cxj, cyj] = centroid(ps);
        centroids(j,:) = [cxj cyj];
        U_peaks = U_peaks + Aobs*G2d(cxj,cyj,sigObs);
    end
    Agoal   = 0.80*max(U_base(:));
    sigGoal = 0.06*GS;
    U_goalSink = -Agoal * G2d(x_goal,y_goal,sigGoal);

    U_vis = U_base + U_peaks + U_goalSink;

    fig3 = figure('Name','3D Potential Field (peaks & sink)','NumberTitle','off'); clf;
    s = surf(GX,GY,U_vis,'EdgeColor','none'); hold on; box on; grid on;
    shading interp; lighting gouraud; camlight headlight; material dull;
    colormap(fig3,'turbo'); colorbar; view(45,30);
    title('Total Potential U(x,y): obstacle peaks + goal sink'); xlabel('x'); ylabel('y'); zlabel('U');

    Ztop = max(U_vis(:))*1.02;
    for j=1:numel(obs)
        P = obs{j};
        plot3(P(1,:),P(2,:),Ztop*ones(1,size(P,2)),'r-','LineWidth',1.2);
    end
    plot3(border_points(1,:),border_points(2,:),Ztop*ones(1,size(border_points,2)),'r-','LineWidth',1.2);

    % markers
    Zc = interp2(GX,GY,U_vis,centroids(:,1),centroids(:,2));
    scatter3(centroids(:,1),centroids(:,2),Zc,36,[1 0 0],'filled');  % obstacle peaks
    Zg = interp2(GX,GY,U_vis,x_goal,y_goal);
    scatter3(x_goal,y_goal,Zg,64,[0 0.4 1],'filled'); text(x_goal,y_goal,Zg,'  waypoint','Color',[0 0.4 1]);

    Z0 = interp2(GX,GY,U_vis,x,y,'linear',Ztop);
    hPath3 = plot3(x,y,Z0,'b-','LineWidth',2);
end

%% ---------------- Simulation loop ----------------
dT=0.1; t=1; t_max=10000;
X = zeros(1,t_max); Y = zeros(1,t_max); X(1)=x; Y(1)=y;

while (hypot(x-x_goal,y-y_goal) > tol_goal) && (t < t_max)

    % Attractive + sink
    rvec = [x y]-[x_goal y_goal]; r = hypot(rvec(1),rvec(2));
    if r <= dstar, g_att = zeta*rvec; else, g_att = zeta*dstar*rvec/max(r,1e-9); end
    g_sink = ksink * ([x y]-[x_goal y_goal]) * (1/sig_sink^2) * exp(-((x-x_goal)^2+(y-y_goal)^2)/(2*sig_sink^2));

    % Repulsion from every obstacle + border (nearest-vertex approx)
    g_rep = [0 0];
    for j=1:numel(obs)
        [idx, dj] = dsearchn(obs{j}', [x y]);
        if dj <= Qstar
            p = obs{j}(:,idx)'; v = ([x y] - p);
            g_rep = g_rep + (eta*(1/Qstar - 1/dj) * 1/max(dj^2,1e-9))*v;
        end
    end
    [ib,dbp] = dsearchn(border_points', [x y]);
    if dbp<=Qstar
        p = border_points(:,ib)'; v = ([x y]-p);
        g_rep = g_rep + (eta*(1/Qstar - 1/dbp) * 1/max(dbp^2,1e-9))*v;
    end

    % Occupancy “mountain”
    gWx = interp2(XC,YC, Wgx, x, y, 'linear', 0);
    gWy = interp2(XC,YC, Wgy, x, y, 'linear', 0);
    g_grid = kg*[gWx, gWy];

    % Total gradient and force
    nablaU = g_att + g_sink + g_rep + g_grid;
    F_goal  = -(g_att + g_sink);
    F_total = -nablaU;

    % Heading & speed
    theta_ref = atan2(F_total(2), F_total(1));
    err = mod(theta_ref - theta + pi, 2*pi) - pi;
    if abs(err) <= error_theta_max
        % slow down near obstacles (safety)
        dmin_all = inf;
        for j=1:numel(obs)
            [~, dj_tmp] = dsearchn(obs{j}', [x y]);
            if dj_tmp < dmin_all, dmin_all = dj_tmp; end
        end
        slow = 0.5 + 0.5*tanh( 6*(dmin_all/Qstar) );         % in (0,1]
        v_ref = min( slow * norm(F_total), v_max );
    else
        v_ref = 0;
    end
    omega_ref = min(max(Kp_omega*err, -omega_max), omega_max);

    % Integrate
    theta = theta + omega_ref*dT;
    x_new = x + v_ref*cos(theta)*dT;
    y_new = y + v_ref*sin(theta)*dT;

    % ---- Hard no-go: if inside any obstacle, project out along repulsion ----
    entered = false;
    for j=1:numel(obs)
        if inpolygon(x_new,y_new,obs{j}(1,:),obs{j}(2,:))
            entered = true; break;
        end
    end
    if entered
        % backtrack and push outward a small step along +g_rep (away from obstacle)
        x_new = x; y_new = y;
        step_out = 0.004*GS * (g_rep / max(norm(g_rep),1e-9));
        x_new = x_new + step_out(1); y_new = y_new + step_out(2);
    end

    % Commit
    x = min(max(x_new,0),GS);  y = min(max(y_new,0),GS);
    t = t+1; X(t)=x; Y(t)=y;

    %% ------------------- DRAW (boat centered) -------------------
    set(hMap,'XData', [0 GS]-x + CENTER(1), 'YData', [0 GS]-y + CENTER(2));
    set(hOcc,'XData', [0 GS]-x + CENTER(1), 'YData', [0 GS]-y + CENTER(2));
    for j=1:numel(obs)
        [ox,oy] = recenter(obs{j}(1,:),obs{j}(2,:),x,y);
        set(hObs(j),'XData',ox,'YData',oy);
    end
    [bx,by] = recenter(border_points(1,:),border_points(2,:),x,y); set(hBrd,'XData',bx,'YData',by);

    [gx_s,gy_s] = recenter(x_goal,y_goal,x,y); set(hGoal,'XData',gx_s,'YData',gy_s);
    [Xp, Yp]    = recenter(X(1:t),Y(1:t),x,y); set(hPath,'XData',Xp,'YData',Yp);
    set(hRef, 'XData',[CENTER(1) CENTER(1)+0.05*GS*cos(theta_ref)], ...
              'YData',[CENTER(2) CENTER(2)+0.05*GS*sin(theta_ref)]);
    set(hHead,'XData',[CENTER(1) CENTER(1)+0.05*GS*cos(theta)], ...
              'YData',[CENTER(2) CENTER(2)+0.05*GS*sin(theta)]);

    % vectors at boat
    sf = 0.15*GS;
    Ft = F_total / max(norm(F_total),1e-9);
    Fg = F_goal  / max(norm(F_goal),1e-9);
    set(hF_tot, 'XData',CENTER(1),'YData',CENTER(2),'UData',sf*Ft(1),'VData',sf*Ft(2));
    set(hF_goal,'XData',CENTER(1),'YData',CENTER(2),'UData',sf*Fg(1),'VData',sf*Fg(2));

    % greedy look-ahead (shows waypoint-finding around obstacles)
    xa=x; ya=y; XpLW=zeros(1,25); YpLW=zeros(1,25);
    for kk=1:25
        % local gradient at (xa,ya)
        rvec = [xa ya]-[x_goal y_goal]; r = hypot(rvec(1),rvec(2));
        if r <= dstar, g_att2 = zeta*rvec; else, g_att2 = zeta*dstar*rvec/max(r,1e-9); end
        g_sink2 = ksink*([xa ya]-[x_goal y_goal])*(1/sig_sink^2)*exp(-((xa-x_goal)^2+(ya-y_goal)^2)/(2*sig_sink^2));
        g_rep2=[0 0];
        for j=1:numel(obs)
            [idj, dj] = dsearchn(obs{j}', [xa ya]);
            if dj<=Qstar, p=obs{j}(:,idj)'; g_rep2 = g_rep2 + (eta*(1/Qstar-1/dj)/max(dj^2,1e-9))*([xa ya]-p); end
        end
        [ibb,dbb] = dsearchn(border_points', [xa ya]);
        if dbb<=Qstar, p=border_points(:,ibb)'; g_rep2=g_rep2+(eta*(1/Qstar-1/dbb)/max(dbb^2,1e-9))*([xa ya]-p); end
        gWx = interp2(XC,YC, Wgx, xa, ya, 'linear', 0);
        gWy = interp2(XC,YC, Wgy, xa, ya, 'linear', 0);
        g_tot2 = g_att2 + g_sink2 + g_rep2 + kg*[gWx,gWy];

        step = LOOKAHEAD_STEP * g_tot2 / max(norm(g_tot2),1e-9);
        xa = xa - step(1); ya = ya - step(2);
        XpLW(kk)=xa; YpLW(kk)=ya;
    end
    [wx,wy] = recenter(xa,ya,x,y); set(hWP,'XData',wx,'YData',wy);
    [wpx,wpy] = recenter(XpLW,YpLW,x,y); set(hWPath,'XData',wpx,'YData',wpy);

    drawnow;

    % 3-D path extension
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
