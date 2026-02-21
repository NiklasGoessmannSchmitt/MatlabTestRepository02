%% UKF-Frenet: Schätzung der Krümmung kappa aus run2-Messdaten
% + Figure 1: CSV-Übersicht (3x2) – alle 6 Spalten
% + Figure 2: UKF-Zeitverläufe (s, d, theta_e, kappa)
% - Messungen aus CSV:
%     * d              : trackDeviation.y [mm]  -> [m]
%     * theta_e        : trackDeviation.a [deg] -> [rad]
%     * s (NEU)        : frontPGV.xPosition [mm] -> [m]
% - Eingänge aus CSV: vCurrent.x [mm/s] -> [m/s], vCurrent.a [deg/s] -> [rad/s]
% - Export analog run123_csv_einlesen_003_export_02 (PDF/FIG/EPS; PNG optional)

clc; close all;

%% ---------------- Konfiguration ----------------
csvRun = 'FrontPGVdrive-run2 (1).csv'; % Messdatei (run2)
%csvRun = 'FrontPGVdrive-CCW.csv';
l_m = 0.3516; % Sensorpunkt P=[l;0] in m
useNoiseCase = 14; % wähle Case 2..15 (R und Q werden im Case erzeugt)

% Export-Konfiguration (analog deiner Export-Routine)
outDir = fullfile(pwd, 'fig-export');
if ~exist(outDir,'dir'), mkdir(outDir); end
DO_PDF = false; DO_PNG = false; DO_FIG = false; DO_EPS = false;

%% ---------------- CSV einlesen & Zeitbasis ----------------
[T, t_s, Ts_nom] = readRun(csvRun);
fprintf('Zeitbasis: N=%d, median(Ts)=%.6f s (Timing-Jitter toleriert)\n', numel(t_s), Ts_nom);

%% ========== Figure 1: CSV-Übersicht 3x2 ==========
figOverview = figure('Color','w', 'Name','CSV-Übersicht – 6 Spalten (run2)');
tl = tiledlayout(figOverview, 3, 2, 'Padding','compact', 'TileSpacing','compact');

% Spaltennamen gemäß CSV
col_xDev = "InstAGV.instMoveCtrlDiff.trackValues.trackDeviation.x"; % [mm]
col_yDev = "InstAGV.instMoveCtrlDiff.trackValues.trackDeviation.y"; % [mm]
col_aDev = "InstAGV.instMoveCtrlDiff.trackValues.trackDeviation.a"; % [deg]
col_xPos = "frontPGV.xPosition";                                    % [mm]
col_vX   = "InstAGV.instMoveCtrlDiff.vCurrent.x";                   % [mm/s]
col_vA   = "InstAGV.instMoveCtrlDiff.vCurrent.a";                   % [deg/s]
req = [col_xDev, col_yDev, col_aDev, col_xPos, col_vX, col_vA];
vn = T.Properties.VariableNames;
miss = req(~ismember(req, vn));
if ~isempty(miss), error('Fehlende Spalten: %s', strjoin(miss, ', ')); end

xDev = T.(col_xDev); yDev = T.(col_yDev); aDev = T.(col_aDev);
xPos = T.(col_xPos); vX = T.(col_vX); vA = T.(col_vA);

% (1) X-Abweichung
ax1 = nexttile(tl, 1); plot(t_s, xDev, 'b'); grid on;
xlabel('Zeit [s]'); ylabel('trackDeviation.x [mm]'); title('Spurabweichung X');

% (2) Y-Abweichung
ax2 = nexttile(tl, 2); plot(t_s, yDev, 'r'); grid on;
xlabel('Zeit [s]'); ylabel('trackDeviation.y [mm]'); title('Spurabweichung Y');

% (3) Winkelabweichung
ax3 = nexttile(tl, 3); plot(t_s, aDev, 'k'); grid on;
xlabel('Zeit [s]'); ylabel('trackDeviation.a [deg]'); title('Spurabweichung A (Winkel)');

% (4) x-Position
ax4 = nexttile(tl, 4); plot(t_s, xPos, 'm'); grid on;
xlabel('Zeit [s]'); ylabel('frontPGV.xPosition [mm]'); title('Front-PGV: x-Position');

% (5) v(t)
ax5 = nexttile(tl, 5); plot(t_s, vX, 'g'); grid on;
xlabel('Zeit [s]'); ylabel('vCurrent.x [mm/s]'); title('Geschwindigkeit X');

% (6) omega(t)
ax6 = nexttile(tl, 6); plot(t_s, vA, 'c'); grid on;
xlabel('Zeit [s]'); ylabel('vCurrent.a [deg/s]'); title('Winkelgeschwindigkeit A');

linkaxes([ax1, ax2, ax3, ax4, ax5, ax6], 'x');
sgtitle(tl, 'FrontPGV – Run 2 (mit Einheiten)');

% Export der 3x2-Übersicht
stamp = datestr(now, 'yyyymmdd_HHMMSS');
baseOvw = fullfile(outDir, sprintf('CSV_run2_Overview_3x2_%s', stamp));
exportFig3x2(figOverview, baseOvw, DO_PDF, DO_PNG, DO_FIG, DO_EPS);

%% ---------------- Messungen & Eingänge (konvertiert) ----------------
[d_meas_m, theta_e_meas_rad, xPos_mm] = extractMeasurements(T); % mm->m, deg->rad
[v_seq, omega_seq] = extractInputsFromCSV(T, t_s);               % mm/s->m/s, deg/s->rad/s

% NEU: s-Messung aus frontPGV.xPosition (mm) -> [m]
s_meas_m = xPos_mm / 1000;

%% --- Offset-Diagnose für d auf (nahe) Geraden (wie v9) ----------------
theta_deg = rad2deg(theta_e_meas_rad);
is_straight = (abs(omega_seq) < 0.02) & (abs(theta_deg) < 1.5); % Heuristik
d_straight = d_meas_m(is_straight);
if ~isempty(d_straight)
    d_offset_m = median(d_straight, 'omitnan'); % robust gegen Ausreißer
else
    d_offset_m = 0;
end
fprintf('Geschätzter d-Offset aus Geraden: %.3f mm\n', d_offset_m*1e3);
d_meas_m_corr = d_meas_m - d_offset_m; % Korrektur anwenden

%% ---------------- R/Q/UKF-Parameter ----------------
[R, Q, params] = makeNoise(useNoiseCase); % R ist jetzt 3x3 (für [s; d; theta_e])
alpha = params.alpha; beta = params.beta; kappaUKF = params.kappaUKF;
disp('R ='); disp(R);
disp('Q ='); disp(Q);

%% ---------------- UKF-Initialisierung ----------------
% x = [s; d; theta_e; kappa]
x_hat = [ s_meas_m(1); d_meas_m(1); theta_e_meas_rad(1); 0 ];
P = diag([1, 0.1, (5*pi/180)^2, 1e-3]);
nx = 4;
x_hist = zeros(nx, numel(t_s));
x_hist(:,1) = x_hat;

%% ---------------- UKF-Hauptschleife ----------------
for k = 1:numel(t_s)-1
    Ts_k = t_s(k+1) - t_s(k);
    u_k = [v_seq(k); omega_seq(k)];

    f_handle = @(x) fstate_frenet(x, u_k, Ts_k, l_m);
    h_handle = @(x) hmeas_frenet(x);    % gibt [s; d; theta_e] zurück


    % Messvektor: NEU mit s-Meas (ohne d-Offset-Korrektur)
    z_k = [ s_meas_m(k+1); d_meas_m(k+1); theta_e_meas_rad(k+1) ];     %d_meas_m_corr(k+1);

    [x_hat, P] = ukf_step(f_handle, x_hat, P, h_handle, z_k, Q, R, alpha, beta, kappaUKF);
    x_hist(:,k+1) = x_hat;
end

%% ========== Figure 2: UKF-Zeitverläufe (s, d, theta_e, kappa) ==========
figUKF = figure('Color','w','Name','Frenet-UKF');
t = t_s;
tl2 = tiledlayout(figUKF, 4, 1, 'Padding','compact', 'TileSpacing','compact');

% s(t) – jetzt mit Messung
nexttile(tl2, 1);
plot(t, s_meas_m, 'g', t, x_hist(1,:), 'k'); grid on; ylabel('s [m]');
legend('Messung s','UKF','Location','best');

% d(t)
nexttile(tl2, 2);
plot(t, d_meas_m, 'g', t, x_hist(2,:), 'k'); grid on;
ylabel('d [m]'); legend('Messung d','UKF','Location','best');

% theta_e(t)
nexttile(tl2, 3);
plot(t, rad2deg(theta_e_meas_rad), 'g', t, rad2deg(x_hist(3,:)), 'k'); grid on;
ylabel('\theta_e [deg]'); legend('Messung \theta_e','UKF','Location','best');

% kappa(t)
nexttile(tl2, 4);
plot(t, x_hist(4,:), 'm'); grid on; ylabel('\kappa [1/m]'); xlabel('t [s]');

sgtitle(tl2, sprintf(['Frenet-UKF \\kappa Schätzung - NoiseCase:%d, ' ...
    'R=diag(%.4g %.4g %.4g), Q=diag(%.4g %.4g %.4g %.4g)'], ...
    useNoiseCase, R(1,1), R(2,2), R(3,3), Q(1,1), Q(2,2), Q(3,3), Q(4,4)));

% Export der UKF-Zeitverlaufs-Figur
baseUKF = fullfile(outDir, sprintf('UKF_run2_TimeSeries_%s', stamp));
exportFigTall(figUKF, baseUKF, DO_PDF, DO_PNG, DO_FIG, DO_EPS);

%% ==================== Lokale Hilfsfunktionen ====================
function [T, t_s, Ts_nom] = readRun(csvRun)
    if ~exist(csvRun,'file'), error('CSV nicht gefunden: %s', csvRun); end
    opts = detectImportOptions(csvRun,'NumHeaderLines',0);
    opts.VariableNamingRule = 'preserve';
    T = readtable(csvRun, opts);
    vn = T.Properties.VariableNames;
    iTime = find(contains(vn,"X(ms"),1);
    if isempty(iTime), error('Zeitspalte "X(ms)" nicht gefunden.'); end
    t_ms = T.(vn{iTime});
    t_s = (t_ms - t_ms(1))/1000; % Sekunden ab Messstart
    Ts_nom = median(diff(t_s)); % nominales Ts
end

function [d_m, theta_rad, xPos_mm] = extractMeasurements(T)
    d_m = T.("InstAGV.instMoveCtrlDiff.trackValues.trackDeviation.y") / 1000; % mm->m
    theta_rad = deg2rad(T.("InstAGV.instMoveCtrlDiff.trackValues.trackDeviation.a")); % deg->rad
    if any(strcmp(T.Properties.VariableNames,"frontPGV.xPosition"))
        xPos_mm = T.("frontPGV.xPosition");
    else
        xPos_mm = nan(height(T),1);
    end
end

function [v_seq, omega_seq] = extractInputsFromCSV(T, t_s)
    assert(any(strcmp(T.Properties.VariableNames,"InstAGV.instMoveCtrlDiff.vCurrent.x")), ...
        'Spalte vCurrent.x fehlt in CSV.');
    assert(any(strcmp(T.Properties.VariableNames,"InstAGV.instMoveCtrlDiff.vCurrent.a")), ...
        'Spalte vCurrent.a fehlt in CSV.');
    v_seq = T.("InstAGV.instMoveCtrlDiff.vCurrent.x") / 1000; % m/s
    omega_seq = deg2rad(T.("InstAGV.instMoveCtrlDiff.vCurrent.a")); % rad/s
    % Sicherheit: Länge an Zeitvektor anpassen, ggf. interpolieren:
    if numel(v_seq) ~= numel(t_s)
        v_seq = interp1(linspace(t_s(1),t_s(end),numel(v_seq)), v_seq, t_s, 'linear','extrap');
    end
    if numel(omega_seq) ~= numel(t_s)
        omega_seq = interp1(linspace(t_s(1),t_s(end),numel(omega_seq)), omega_seq, t_s, 'linear','extrap');
    end
end

function [R, Q, params] = makeNoise(caseID)
% Erzeugt R (JETZT 3x3 für [s; d; theta_e]) und Q (4x4) pro Case.
% Vorbetrachtung (aus v9): Sensor max Auflösung (d, theta_e) = (0.1mm, 0.1deg)
    d_Aufloesung = 0.1 /1000;          % [m]
    thetae_Auflosung = deg2rad(0.1);   % [rad]      thetae_Aufloesung
    s_Aufloesung = 1.0 /1000;          % [m] pragmatische Annahme: 1 mm Quantisierung für xPosition     1.0 /1000;

    % Default-Std:
    sigma_R_d      = d_Aufloesung / sqrt(12);
    sigma_R_thetae = thetae_Auflosung / sqrt(12);
    sigma_R_s      = s_Aufloesung / sqrt(12);

    switch caseID
        case 2
            % --- Messrauschen ---
            sigma_R_d      = 0.002;  % [m]
            sigma_R_thetae = 0.005;  % [rad]
            sigma_R_s      = sigma_R_d;  % pragmatisch: gleiche Größenordnung wie d
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);

            % --- Prozessrauschen Q ---
            sigma_Q_s      = 0.010 * 0.1;
            sigma_Q_d      = 0.010 * 0.1;
            sigma_Q_thetae = 0.800 * 0.1;
            sigma_Q_kappa  = 0.700 * 0.1;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        case 3
            sigma_R_d = 0.002; sigma_R_thetae = 0.005; sigma_R_s = sigma_R_d;
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);
            sigma_Q_s=0.010*0.1; sigma_Q_d=0.010*0.1; sigma_Q_thetae=0.300*0.1; sigma_Q_kappa=0.700*0.1;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        case 4
            sigma_R_d = 0.002; sigma_R_thetae = 0.005; sigma_R_s = sigma_R_d;
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);
            sigma_Q_s=0.010*0.1; sigma_Q_d=0.005*0.1; sigma_Q_thetae=0.300*0.1; sigma_Q_kappa=0.700*0.1;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        case 5
            sigma_R_d = 0.002; sigma_R_thetae = 0.005; sigma_R_s = sigma_R_d;
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);
            sigma_Q_s=0.010*0.1; sigma_Q_d=0.015*0.1; sigma_Q_thetae=0.300*0.1; sigma_Q_kappa=0.700*0.1;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        case 6
            sigma_R_d = 0.002; sigma_R_thetae = 0.005; sigma_R_s = sigma_R_d;
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);
            sigma_Q_s=0.010*0.1; sigma_Q_d=0.0098*0.1; sigma_Q_thetae=0.300*0.1; sigma_Q_kappa=0.700*0.1;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        case 7
            sigma_R_d = 0.002; sigma_R_thetae = 0.005; sigma_R_s = sigma_R_d;
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);
            sigma_Q_s=0.010*0.1; sigma_Q_d=0.0098*0.1; sigma_Q_thetae=0.500*0.1; sigma_Q_kappa=12.900*0.1;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        case 8
            sigma_R_d = 0.009; sigma_R_thetae = 0.009; sigma_R_s = sigma_R_d;
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);
            sigma_Q_s=0.010*0.1; sigma_Q_d=0.009*0.1; sigma_Q_thetae=0.006*0.1; sigma_Q_kappa=0.0900*0.1;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        case 9
            sigma_R_d = 0.007; sigma_R_thetae = 0.08; sigma_R_s = sigma_R_d;
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);
            sigma_Q_s=0.100*0.1; sigma_Q_d=0.300*0.1; sigma_Q_thetae=0.060*0.1; sigma_Q_kappa=0.400*0.1;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        case 10
            sigma_R_d = 0.001; sigma_R_thetae = 0.001; sigma_R_s = 0.001;
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);
            sigma_Q_s=0.010; sigma_Q_d=0.030; sigma_Q_thetae=0.006; sigma_Q_kappa=0.400;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        case 11
            sigma_R_d      = d_Aufloesung / sqrt(12);
            sigma_R_thetae = thetae_Auflosung / sqrt(12);
            sigma_R_s      = s_Aufloesung / sqrt(12); % 1 mm Annahme
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);
            sigma_Q_s=0.0005; sigma_Q_d=sigma_R_d; sigma_Q_thetae=sigma_R_thetae; sigma_Q_kappa=0.004;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        case 12
            sigma_R_d      = d_Aufloesung / sqrt(12);
            sigma_R_thetae = thetae_Auflosung / sqrt(12);
            sigma_R_s      = s_Aufloesung / sqrt(12);
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);
            sigma_Q_s=5e-4; sigma_Q_d=4*sigma_R_d; sigma_Q_thetae=4*sigma_R_thetae; sigma_Q_kappa=0.04;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        case 13
            sigma_R_d      = d_Aufloesung / sqrt(12);
            sigma_R_thetae = thetae_Auflosung / sqrt(12);
            sigma_R_s      = s_Aufloesung / sqrt(12);
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);
            sigma_Q_s=5e-4; sigma_Q_d=0.4*sigma_R_d; sigma_Q_thetae=0.8*sigma_R_thetae; sigma_Q_kappa=0.003;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        case 14
            sigma_R_d      = d_Aufloesung / sqrt(12);      % ~2.89e-05 m
            sigma_R_thetae = thetae_Auflosung / sqrt(12); % ~5.04e-04 rad
            sigma_R_s      = s_Aufloesung / sqrt(12);      % 1mm -> ~2.89e-04 m
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);
            sigma_Q_s=5*sigma_R_s; sigma_Q_d=0.4*sigma_R_d; sigma_Q_thetae=0.8*sigma_R_thetae; sigma_Q_kappa=0.9*0.0025;   %sigma_Q_s=5e-4;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        case 15
            sigma_R_d      = 1*d_Aufloesung / sqrt(12);
            sigma_R_thetae = sigma_R_d;
            sigma_R_s      = s_Aufloesung / sqrt(12);
            R = diag([sigma_R_s^2, sigma_R_d^2, sigma_R_thetae^2]);
            sigma_Q_s=5e-4; sigma_Q_d=0.4*sigma_R_d; sigma_Q_thetae=0.8*sigma_R_thetae; sigma_Q_kappa=0.1*0.0025;
            Q = diag([sigma_Q_s^2, sigma_Q_d^2, sigma_Q_thetae^2, sigma_Q_kappa^2]);

        otherwise
            error('Unbekannter noise_case.');
    end

    % UKF-Skalierung (Merwe-Scaled)
    params.alpha = 1e-3;
    params.beta = 2;
    params.kappaUKF = 0;
end

function xnext = fstate_frenet(x, u, Ts, l)
% Frenet-Dynamik (diskret, Euler): x=[s; d; theta_e; kappa], u=[v; omega]
    s = x(1); d = x(2); theta_e = x(3); kappa = x(4);
    v = u(1); omega = u(2);
    denom = 1 - kappa*d;
    if abs(denom) < 1e-6
        denom = 1e-6 * sign(denom + (denom==0)); % Schutz gegen Singularität
    end
    s_dot     = (v*cos(theta_e) - l*omega*sin(theta_e)) / denom;
    d_dot     =  v*sin(theta_e) + l*omega*cos(theta_e);
    theta_dot =  omega - kappa*s_dot;
    kappa_dot =  0; % Random Walk via Prozessrauschen
    xnext = x + Ts*[s_dot; d_dot; theta_dot; kappa_dot];
end

function z = hmeas_frenet(x)
% NEU: Messfunktion liefert [s; d; theta_e]
    z = [x(1); x(2); x(3)];
end

function [x,P] = ukf_step(fstate, x, P, hmeas, z, Q, R, alpha, beta, kappaUKF)
    L = numel(x);
    lambda = alpha^2*(L + kappaUKF) - L;
    c = L + lambda;
    Wm = [lambda/c, 0.5/c*ones(1,2*L)];
    Wc = Wm; Wc(1) = Wc(1) + (1 - alpha^2 + beta);

    % Robuste Cholesky-Zerlegung
    try
        A = chol(c*P, 'lower');
    catch
        A = chol(c*(P + 1e-9*eye(L)), 'lower');
    end
    Xsigma = [x, x + A, x - A];

    % Zeitupdate
    Lsigma = size(Xsigma,2);
    Xpred = zeros(L, Lsigma);
    for i=1:Lsigma, Xpred(:,i) = fstate(Xsigma(:,i)); end
    x_pred = Xpred*Wm';
    X1 = Xpred - x_pred;
    P_pred = X1*diag(Wc)*X1' + Q;

    % Messupdate
    nz = numel(z);
    Zsig= zeros(nz, Lsigma);
    for i=1:Lsigma, Zsig(:,i) = hmeas(Xpred(:,i)); end
    z_pred = Zsig*Wm';
    Z1 = Zsig - z_pred;
    Pz = Z1*diag(Wc)*Z1' + R;
    Pxz = X1*diag(Wc)*Z1';
    K = Pxz / Pz;
    x = x_pred + K*(z - z_pred);
    P = P_pred - K*Pz*K';
end

%% ---------------- Export-Helfer ----------------
function exportFig3x2(figHandle, basePath, doPDF, doPNG, doFIG, doEPS)
    % 3x2-Übersicht: 18 x 15 cm (analog deiner Datei)
    set(figHandle,'PaperUnits','centimeters','PaperPosition',[0 0 18 15],'PaperSize',[18 15]);
    set(figHandle,'InvertHardcopy','off'); set(figHandle,'Renderer','painters');
    if doEPS, print(figHandle,'-depsc','-painters','-r300',[basePath '.eps']); end
    if doPDF, print(figHandle,'-dpdf','-painters',[basePath '.pdf']); end
    if doPNG, print(figHandle,'-dpng','-r300',[basePath '.png']); end
    if doFIG, savefig(figHandle,[basePath '.fig']); end
end

function exportFigTall(figHandle, basePath, doPDF, doPNG, doFIG, doEPS)
    % Hochformat (4x1-Zeitverläufe): 18 x 12 cm
    set(figHandle,'PaperUnits','centimeters','PaperPosition',[0 0 18 12],'PaperSize',[18 12]);
    set(figHandle,'InvertHardcopy','off'); set(figHandle,'Renderer','painters');
    if doEPS, print(figHandle,'-depsc','-painters','-r300',[basePath '.eps']); end
    if doPDF, print(figHandle,'-dpdf','-painters',[basePath '.pdf']); end
    if doPNG, print(figHandle,'-dpng','-r300',[basePath '.png']); end
    if doFIG, savefig(figHandle,[basePath '.fig']); end
end
