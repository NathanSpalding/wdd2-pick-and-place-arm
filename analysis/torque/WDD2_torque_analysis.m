%% WDD2 pick-and-place arm: static joint torque analysis with a 500 g payload
% =========================================================================
% Author : Nathan Spalding (WDD2, Mechatronic Project 478)
% Purpose: Find the maximum holding torque at each joint over the whole
%          workspace, using the link masses and geometry exported from the
%          CAD model (WDD2_arm.urdf), and compare it with the servo ratings.
%
% Method
%   * Forward kinematics are built from the URDF joint origins and axes, so
%     the small lateral offsets of the real design are included.
%   * The holding (gravity) torque about each pitch joint is the sum over all
%     links beyond that joint of  -[(c_k - p_i) x m_k*g] . a_i , plus the
%     same term for the payload, which is a point mass at the gripping
%     point (TCP, 133 mm beyond the wrist axis).
%   * The shoulder, elbow and wrist are swept over their full joint limits.
%     Poses that put the tip or the wrist below the floor are discarded.
%   * The base axis is vertical, so gravity gives it no holding torque; it
%     is sized on the torque needed to accelerate the arm's inertia.
%   * A safety factor (SF) covers acceleration, friction and model error.
%
% Outputs
%   * Results tables in the Command Window
%   * Figures (PNG) in ./figures
%   * KEY FINDINGS printed at the end and saved to key_findings.txt
%
% Source data: WDD2_arm.urdf (fusion2urdf export of the Fusion 360 model,
% CAD masses, i.e. solid-modelled parts). Servo ratings at the supply
% voltage of each rail (see the power budget).
% =========================================================================
clear; close all; clc;

%% 1. Settings  (edit these)
PAYLOAD_KG       = 0.5;      % payload in the gripper (kg)
SF               = 1.5;      % safety factor on torque
G                = 9.81;     % gravity (m/s^2)
TARGET_REACH_MM  = 300;      % original design target reach (mm)
CUBE_Z_MM        = 25;       % pick height: centre of a 50 mm cube (mm)
STEP_DEG         = 2.5;      % sweep resolution (deg). 2.5 deg takes about
                             % 0.5-1 min in MATLAB; use 5 for a quick run
BASE_ACCEL       = 8.0;      % base angular acceleration limit (rad/s^2),
                             % same as the simulator's default
FIG_DIR          = 'figures';

% Servo ratings at the rail voltages used in the arm
%            name        servo            rated torque (N.m)  rail (V)
SERVO = struct( ...
  'joint', {'Base','Shoulder','Elbow','Wrist'}, ...
  'servo', {'DS3240-270-FB','DS5180-270-FB','DS3240-270-FB','DS3225'}, ...
  'rated', {3.9, 9.8, 4.4, 2.45}, ...
  'volts', {6.7, 8.0, 6.7, 6.7});

% Earlier hand calculation for the 300 mm target (torques incl. SF = 1.5),
% kept for comparison in the key findings. [shoulder elbow wrist], N.m
PREV_SF_TORQUE = [5.29 2.58 0.75];

if ~exist(FIG_DIR, 'dir'); mkdir(FIG_DIR); end

%% 2. Arm model from WDD2_arm.urdf
% Link frames are the Fusion 360 frames exported by fusion2urdf (Y up in
% Fusion). The fixed world_to_base joint turns them to Z up, with the arm
% pointing along +X at zero joint angles.
arm.world_xyz = [0.475 0.029124 0.0641];        % world_to_base origin (m)
arm.world_rpy = [1.5708 0 -1.5708];             % world_to_base rpy (rad)

% joint i: origin in parent frame (m), axis (in own frame), limits (deg)
arm.jxyz  = [ 0.029124 -0.028703  0.475;       % base
             -0.012725  0.0532   -0.058245;    % shoulder
              0.002851  3.4e-05  -0.175;       % elbow
              0.00275   3.4e-05  -0.144999];   % wrist
arm.jaxis = [0 -1 0; 1 0 0; 1 0 0; 1 0 0];
arm.jlo   = [-70 -30 -90 -85];
arm.jhi   = [ 70  75  90 115];
arm.tcp   = [0 0 -0.133];                       % gripping point in wrist link

% links after each joint: mass (kg), centre of mass in link frame (m),
% inertia about the CoM in link frame (kg.m^2): [ixx iyy izz ixy iyz ixz]
arm.lname = {'turntable','upper arm','forearm','gripper'};
arm.mass  = [0.456944 0.229105 0.209680 0.185427];
arm.com   = [0.010355 0.034853 -0.041212;
             0.018912 0.001420 -0.120493;
             0.018519 0.001425 -0.105553;
             0.008339 0.007155 -0.063747];
arm.inert = [0.000468 0.000512 0.000437  5e-06  -5e-06  7.9e-05;
             0.000777 0.000860 0.000108  0      0       6.5e-05;
             0.000416 0.000501 0.000105  0      0       4.2e-05;
             0.000321 0.000358 9.4e-05   4e-06  -1e-05  3e-06];
m_base_link = 2.542637;                         % fixed base (does not move)

fprintf('WDD2 arm model (from WDD2_arm.urdf)\n');
fprintf('  Moving mass: %.0f g  (+ %.0f g payload)\n', ...
        1000*sum(arm.mass), 1000*PAYLOAD_KG);
fprintf('  Total arm mass incl. base: %.2f kg\n\n', sum(arm.mass) + m_base_link);

%% 3. Check the model: reach at zero pose
k0 = arm_fk(arm, [0 0 0 0]);
reach_max_mm   = 1000*hypot(k0.tcp(1), k0.tcp(2));
reach_sh_mm    = 1000*norm(k0.tcp - k0.p(:,2));
fprintf('Geometry (all joints at 0 = arm stretched out horizontally)\n');
fprintf('  Shoulder axis height above floor : %.0f mm\n', 1000*k0.p(3,2));
fprintf('  Tip reach from base axis         : %.0f mm\n', reach_max_mm);
fprintf('  Tip distance from shoulder axis  : %.0f mm\n', reach_sh_mm);
fprintf('  Original target reach            : %.0f mm\n\n', TARGET_REACH_MM);

%% 4. Sweep the whole joint space
qs = arm.jlo(2):STEP_DEG:arm.jhi(2);
qe = arm.jlo(3):STEP_DEG:arm.jhi(3);
qw = arm.jlo(4):STEP_DEG:arm.jhi(4);
n  = numel(qs)*numel(qe)*numel(qw);
fprintf('Sweeping %d poses (step %.1f deg)...\n', n, STEP_DEG);

R   = zeros(n,1);  Z = zeros(n,1);  Q = zeros(n,3);  TAU = zeros(n,3);
OK  = false(n,1);
row = 0; tic;
for a = qs
  for b = qe
    for c = qw
      row = row + 1;
      q = [0 a b c];
      k = arm_fk(arm, q);
      t = holding_torque(arm, k, PAYLOAD_KG, G);
      Q(row,:)   = [a b c];
      TAU(row,:) = t(2:4);
      R(row)     = 1000*hypot(k.tcp(1), k.tcp(2));
      Z(row)     = 1000*k.tcp(3);
      OK(row)    = k.tcp(3) >= 0 && k.p(3,4) >= 0;   % tip and wrist above floor
    end
  end
end
fprintf('  done in %.1f s; %d poses are above the floor.\n\n', toc, nnz(OK));

Qv = Q(OK,:);  TAUv = TAU(OK,:);  Rv = R(OK);  Zv = Z(OK);

%% 5. Worst case over the reachable workspace
names = {'Shoulder','Elbow','Wrist'};
rated = [SERVO(2:4).rated];
fprintf('Worst-case holding torque, %.0f g payload (whole workspace)\n', 1000*PAYLOAD_KG);
fprintf('  %-9s %8s %10s %9s %10s %8s   %s\n', 'Joint', 'Static', 'x SF', ...
        'Rated', 'Utilised', 'Margin', 'Worst pose [sh el wr] (deg)');
worst = struct();
k0  = arm_fk(arm, [0 0 0 0]);                 % arm stretched out horizontally
tz  = abs(holding_torque(arm, k0, PAYLOAD_KG, G));
for j = 1:3
  [tmax, i] = max(abs(TAUv(:,j)));
  worst(j).tau  = tmax;
  worst(j).q    = Qv(i,:);
  worst(j).r    = Rv(i);
  worst(j).z    = Zv(i);
  if tz(j+1) >= 0.995*tmax                    % ties: report the stretched-out pose
    worst(j).q = [0 0 0];
    worst(j).r = 1000*hypot(k0.tcp(1), k0.tcp(2));
    worst(j).z = 1000*k0.tcp(3);
  end
  worst(j).req  = SF*tmax;
  worst(j).util = worst(j).req / rated(j);
  fprintf('  %-9s %6.2f Nm %7.2f Nm %6.2f Nm %9.0f %% %7.2fx   [%5.1f %5.1f %5.1f]\n', ...
          names{j}, tmax, worst(j).req, rated(j), 100*worst(j).util, ...
          1/worst(j).util, worst(j).q);
end
fprintf('\n');

%% 6. Torque against reach (envelope over working poses at each reach)
% Working poses: forearm and gripper point forwards (within +/-90 deg of
% horizontal). This leaves out folded-back poses with the forearm reaching
% back over the shoulder, which are not used for pick and place. The
% worst-case table above still covers every pose.
fore = Qv(:,1) + Qv(:,2);                 % forearm angle from horizontal
tilt = sum(Qv, 2);                        % gripper angle from horizontal
work = abs(fore) <= 90 & abs(tilt) <= 90;
edges   = 0:10:ceil(max(Rv)/10)*10;
centres = edges(1:end-1) + 5;
env     = nan(numel(centres), 3);
for b = 1:numel(centres)
  in = work & Rv >= edges(b) & Rv < edges(b+1);
  if any(in); env(b,:) = max(abs(TAUv(in,:)), [], 1); end
end
% worst case within +/- 5 mm of the original target reach
inT    = work & abs(Rv - TARGET_REACH_MM) <= 5;
tau300 = max(abs(TAUv(inT,:)), [], 1);

%% 7. Representative poses
% (a) original target task: pick at 300 mm, cube height, gripper pointing down
[qA, okA] = ik_pitch(arm, TARGET_REACH_MM/1000, CUBE_Z_MM/1000, -90, [60 -70 -80]);
% (b) new maximum reach: arm stretched out horizontally
qB = [0 0 0];
% (c) far pick at cube height: steepest gripper angle that still reaches
%     (the same rule as the simulator's Auto gripper mode)
okC = false; tiltC = NaN; qC = [0 0 0];
for tilt = -90:10:0
  [qC, okC] = ik_pitch(arm, 0.480, CUBE_Z_MM/1000, tilt, [20 -40 -20]);
  if okC; tiltC = tilt; break; end
end

poses = {'300 mm pick, gripper down', qA, okA;
         'Max reach, arm horizontal', qB, true;
         sprintf('480 mm pick, gripper %d deg', tiltC), qC, okC};
fprintf('Representative poses (holding torque incl. %.0f g payload)\n', 1000*PAYLOAD_KG);
fprintf('  %-30s %7s %7s %8s %8s %8s\n', 'Pose', 'r (mm)', 'z (mm)', ...
        'Shoulder', 'Elbow', 'Wrist');
poseTau = nan(size(poses,1), 3);
for p = 1:size(poses,1)
  q = [0 poses{p,2}];
  k = arm_fk(arm, q);
  t = holding_torque(arm, k, PAYLOAD_KG, G);
  poseTau(p,:) = abs(t(2:4));
  flag = ''; if ~poses{p,3}; flag = '  (IK did not converge)'; end
  fprintf('  %-30s %7.0f %7.0f %6.2f Nm %6.2f Nm %6.2f Nm%s\n', poses{p,1}, ...
          1000*hypot(k.tcp(1),k.tcp(2)), 1000*k.tcp(3), poseTau(p,:), flag);
end
fprintf('\n');

%% 8. Payload capacity at maximum reach
% Torque rises linearly with payload, so find the slope per kg at the
% worst pose and solve rated/SF = tau_arm + slope*m.
k   = arm_fk(arm, [0 0 0 0]);
t0  = holding_torque(arm, k, 0, G);
t1  = holding_torque(arm, k, 1, G);
slope   = abs(t1(2:4) - t0(2:4));               % N.m per kg of payload
m_cap   = (rated/SF - abs(t0(2:4))) ./ slope;   % kg, per joint
[cap, jcap] = min(m_cap);
fprintf('Payload capacity at maximum reach (with SF = %.1f)\n', SF);
for j = 1:3
  fprintf('  %-9s %5.2f kg\n', names{j}, m_cap(j));
end
fprintf('  -> limited by the %s: %.2f kg\n\n', lower(names{jcap}), cap);

%% 9. Base joint: torque to accelerate the arm
% Moment of inertia about the (vertical) base axis with the arm stretched
% out and carrying the payload, i.e. the largest it can be.
Ibase = base_inertia(arm, [0 0 0 0], PAYLOAD_KG);
tauB  = Ibase * BASE_ACCEL;
alphaMax = SERVO(1).rated / (SF * Ibase);
fprintf('Base joint (vertical axis, no gravity torque)\n');
fprintf('  Inertia about base axis, arm extended + payload: %.4f kg.m^2\n', Ibase);
fprintf('  Torque at %.1f rad/s^2: %.3f Nm (x SF = %.3f Nm), rated %.2f Nm\n', ...
        BASE_ACCEL, tauB, SF*tauB, SERVO(1).rated);
fprintf('  Max angular acceleration with SF: %.0f rad/s^2\n\n', alphaMax);

%% 10. Figures
cols = [0.12 0.47 0.71; 1.00 0.50 0.05; 0.17 0.63 0.17];

% (1) torque against reach
f1 = figure('Name','Torque vs reach','Color','w','Position',[100 100 900 650]);
for j = 1:3
  subplot(3,1,j); hold on; grid on; box on;
  plot(centres, SF*env(:,j), '-', 'Color', cols(j,:), 'LineWidth', 1.8);
  plot(centres, env(:,j), '--', 'Color', cols(j,:), 'LineWidth', 1.0);
  xlim([0 edges(end)]);
  ylim([0 1.15*max(rated(j), SF*max(env(:,j)))]);
  yline_(rated(j), 'k-', sprintf('rated %.2f Nm', rated(j)));
  xline_(TARGET_REACH_MM, 'r:', 'original target');
  xline_(reach_max_mm, 'k:', 'new max reach');
  ylabel('Torque (N\cdotm)');
  title(sprintf('%s (%s)', names{j}, SERVO(j+1).servo));
  if j == 1
    legend({sprintf('max torque x SF %.1f', SF), 'max torque (static)'}, ...
           'Location', 'west');
  end
end
xlabel(['Horizontal reach of the gripping point from the base axis (mm)' ...
        ' - working poses']);
save_fig(f1, fullfile(FIG_DIR, 'torque_vs_reach.png'));

% (2) utilisation of each servo at its worst case (reference label centred)
f2 = figure('Name','Servo utilisation','Color','w','Position',[150 150 600 400]);
u = 100*[worst.util];
bh = bar(u, 0.6); hold on; grid on; box on;
set(bh, 'FaceColor', [0.40 0.55 0.75]);
xlim([0.4 3.6]); ylim([0 max(120, max(u) + 15)]);
yline_(100, 'r-', '100 % of rated torque');
set(gca, 'XTick', 1:3, 'XTickLabel', names);
ylabel(sprintf('Required torque (SF %.1f) / rated torque (%%)', SF));
title(sprintf('Worst-case servo utilisation, %.0f g payload', 1000*PAYLOAD_KG));
for j = 1:3
  text(j, u(j) + 3, sprintf('%.0f %%', u(j)), 'HorizontalAlignment', 'center');
end
save_fig(f2, fullfile(FIG_DIR, 'servo_utilisation.png'));

% (3) side view of the worst-case pose and the original target pose
f3 = figure('Name','Poses','Color','w','Position',[200 200 700 450]);
hold on; grid on; box on; axis equal;
draw_arm(arm, [0 worst(1).q], [0.8 0.1 0.1], 'worst case (shoulder)');
if okA; draw_arm(arm, [0 qA], [0.1 0.4 0.8], '300 mm pick, gripper down'); end
plot([-0.05 0.6], [0 0], 'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
xlabel('x (m)'); ylabel('z (m)'); legend('Location', 'northeast');
title('Side view: worst-case pose vs original target task');
save_fig(f3, fullfile(FIG_DIR, 'poses_side_view.png'));

%% 11. KEY FINDINGS
[~, jlim] = max([worst.util]);
incr = 100*(env_at(centres, env, reach_max_mm) ./ tau300 - 1);
L = {};
L{end+1} = sprintf(['Reach: the as-built arm reaches %.0f mm from the base axis ' ...
  '(stretched out horizontally), %.0f%% beyond the original %.0f mm target.'], ...
  reach_max_mm, 100*(reach_max_mm/TARGET_REACH_MM - 1), TARGET_REACH_MM);
if all(poseTau(2,:) >= 0.995*[worst.tau])
  L{end+1} = sprintf(['Worst case: every pitch joint is most loaded when the arm is ' ...
    'stretched out horizontally (all joints at 0 deg), where the %.0f g payload ' ...
    'and the links have their longest lever arms. Each joint is equally loaded in ' ...
    'any pose where everything beyond it is horizontal.'], 1000*PAYLOAD_KG);
else
  L{end+1} = sprintf(['Worst-case poses [shoulder elbow wrist] (deg): shoulder [%g %g %g], ' ...
    'elbow [%g %g %g], wrist [%g %g %g].'], worst(1).q, worst(2).q, worst(3).q);
end
for j = 1:3
  L{end+1} = sprintf(['%s (%s): %.2f Nm static, %.2f Nm with SF %.1f, against %.2f Nm ' ...
    'rated -> %.0f%% utilised, margin %.2fx.'], names{j}, SERVO(j+1).servo, ...
    worst(j).tau, worst(j).req, SF, rated(j), 100*worst(j).util, 1/worst(j).util);
end
L{end+1} = sprintf(['Limiting joint: the %s is the most heavily loaded servo ' ...
  '(%.0f%% of rated torque at the worst case, including SF %.1f).'], ...
  lower(names{jlim}), 100*worst(jlim).util, SF);
if all([worst.util] <= 1)
  L{end+1} = sprintf(['All three pitch servos hold the %.0f g payload anywhere in the ' ...
    'workspace with the %.1f safety factor applied.'], 1000*PAYLOAD_KG, SF);
else
  L{end+1} = 'WARNING: at least one servo is under-rated for the worst case with the SF applied.';
end
txt = sprintf(['Reach increase vs torque: going from the original %.0f mm target to ' ...
  'the new %.0f mm maximum reach raises the worst-case torque by %.0f%% at the ' ...
  'shoulder and %.0f%% at the elbow.'], TARGET_REACH_MM, reach_max_mm, incr(1:2));
if abs(incr(3)) < 1
  txt = [txt ' The wrist load is unchanged: it depends only on the gripper''s own ' ...
         'angle, not on how far out the arm is.'];
else
  txt = [txt sprintf(' The wrist torque rises by %.0f%%.', incr(3))];
end
L{end+1} = txt;
L{end+1} = sprintf(['Compared with the earlier hand calculation for the %.0f mm target ' ...
  '(%.2f / %.2f / %.2f Nm incl. SF), this CAD-based model gives %.2f / %.2f / %.2f Nm ' ...
  'at %.0f mm and %.2f / %.2f / %.2f Nm at the new maximum reach ' ...
  '(shoulder / elbow / wrist).'], TARGET_REACH_MM, PREV_SF_TORQUE, SF*tau300, ...
  TARGET_REACH_MM, [worst.req]);
L{end+1} = sprintf(['Typical task: picking at %.0f mm with the gripper pointing down ' ...
  'needs only %.2f / %.2f / %.2f Nm (shoulder / elbow / wrist, static), about %.0f%% of ' ...
  'the worst case at the shoulder.'], TARGET_REACH_MM, poseTau(1,:), ...
  100*poseTau(1,1)/worst(1).tau);
L{end+1} = sprintf(['Payload capacity at maximum reach (SF %.1f): %.2f kg, limited by ' ...
  'the %s - %.1fx the %.0f g design payload.'], SF, cap, lower(names{jcap}), ...
  cap/PAYLOAD_KG, 1000*PAYLOAD_KG);
uB = SF*tauB/SERVO(1).rated;
if uB <= 0.5;     wB = 'well within its rating';
elseif uB <= 1;   wB = 'within its rating';
else;             wB = 'OVER its rating - lower the base acceleration';
end
L{end+1} = sprintf(['Base: no gravity torque (vertical axis); it is sized by inertia. ' ...
  'Accelerating the extended, loaded arm (I = %.3f kg.m^2) at %.0f rad/s^2 needs ' ...
  '%.2f Nm with SF %.1f, %.0f%% of the %.1f Nm rating, so the base servo is %s. ' ...
  'The base acceleration should stay below %.0f rad/s^2 with the arm extended.'], ...
  Ibase, BASE_ACCEL, SF*tauB, SF, 100*uB, SERVO(1).rated, wB, alphaMax);
L{end+1} = ['Assumptions: static holding torques from CAD masses (solid-modelled ' ...
  'printed parts, so conservative); servo ratings at the rail voltages; dynamics, ' ...
  'friction and backlash covered by the safety factor; self-collision not checked.'];

fprintf('=====================================================================\n');
fprintf('KEY FINDINGS\n');
fprintf('=====================================================================\n');
fid = fopen('key_findings.txt', 'w');
for i = 1:numel(L)
  fprintf('%2d. %s\n', i, L{i});
  fprintf(fid, '%d. %s\n', i, L{i});
end
fclose(fid);
fprintf('\nSaved: key_findings.txt and figures in ./%s\n', FIG_DIR);


%% ========================================================================
%  Local functions
%  ========================================================================
function k = arm_fk(arm, qdeg)
% Forward kinematics. Returns joint positions p (3x4), joint axes in world
% a (3x4), link CoMs c (3x4), link rotations Rl (3x3x4) and the TCP.
  T = [rpy2r(arm.world_rpy) arm.world_xyz(:); 0 0 0 1];
  k.p = zeros(3,4); k.a = zeros(3,4); k.c = zeros(3,4); k.Rl = zeros(3,3,4);
  for i = 1:4
    T = T * [eye(3) arm.jxyz(i,:)'; 0 0 0 1];             % joint origin
    k.p(:,i) = T(1:3,4);
    k.a(:,i) = T(1:3,1:3) * arm.jaxis(i,:)';
    T = T * [axang(arm.jaxis(i,:), deg2rad(qdeg(i))) [0;0;0]; 0 0 0 1];
    k.Rl(:,:,i) = T(1:3,1:3);
    k.c(:,i) = T(1:3,4) + T(1:3,1:3) * arm.com(i,:)';
  end
  k.tcp = T(1:3,4) + T(1:3,1:3) * arm.tcp(:);
end

function tau = holding_torque(arm, k, payload, g)
% Torque each joint must apply to hold the pose against gravity (N.m).
  F = [0; 0; -g];
  tau = zeros(1,4);
  for i = 1:4
    s = 0;
    for j = i:4                                          % links beyond joint i
      s = s + dot(cross(k.c(:,j) - k.p(:,i), arm.mass(j)*F), k.a(:,i));
    end
    s = s + dot(cross(k.tcp - k.p(:,i), payload*F), k.a(:,i));
    tau(i) = -s;
  end
end

function I = base_inertia(arm, qdeg, payload)
% Moment of inertia of the moving arm (+ point payload) about the base axis.
  k = arm_fk(arm, qdeg);
  a = k.a(:,1) / norm(k.a(:,1));
  p = k.p(:,1);
  I = 0;
  for j = 1:4
    v = arm.inert(j,:);
    Ic = [v(1) v(4) v(6); v(4) v(2) v(5); v(6) v(5) v(3)];
    Iw = k.Rl(:,:,j) * Ic * k.Rl(:,:,j)';
    d  = k.c(:,j) - p;  dperp = d - dot(d,a)*a;
    I  = I + a'*Iw*a + arm.mass(j)*sum(dperp.^2);
  end
  d = k.tcp - p;  dperp = d - dot(d,a)*a;
  I = I + payload*sum(dperp.^2);
end

function [q, ok] = ik_pitch(arm, r, z, tilt_deg, q0)
% Solve shoulder/elbow/wrist (deg) so the TCP is at horizontal reach r and
% height z (m) with gripper tilt = shoulder+elbow+wrist = tilt_deg.
  q = q0; ok = false;
  for it = 1:200
    f = ik_resid(arm, q, r, z, tilt_deg);
    if norm(f(1:2)) < 1e-5 && abs(f(3)) < 1e-3; ok = true; break; end
    J = zeros(3); h = 1e-4;
    for i = 1:3
      dq = zeros(1,3); dq(i) = h;
      J(:,i) = (ik_resid(arm, q + dq, r, z, tilt_deg) - f) / h;
    end
    step = -(J'*J + 1e-6*eye(3)) \ (J'*f);
    q = min(max(q + step', arm.jlo(2:4)), arm.jhi(2:4));
  end
end

function f = ik_resid(arm, q, r, z, tilt_deg)
% Residual for ik_pitch: [reach error; height error; tilt error (rad)].
  k = arm_fk(arm, [0 q]);
  f = [hypot(k.tcp(1), k.tcp(2)) - r;  k.tcp(3) - z;  deg2rad(sum(q) - tilt_deg)];
end

function draw_arm(arm, qdeg, col, label)
% Side view (x-z) of the arm: joints, links and the payload point.
  k = arm_fk(arm, qdeg);
  P = [k.p(:,2:4) k.tcp];
  plot(P(1,:), P(3,:), '-o', 'Color', col, 'LineWidth', 2.5, ...
       'MarkerFaceColor', col, 'DisplayName', label);
  plot(k.tcp(1), k.tcp(3), 's', 'Color', col, 'MarkerSize', 10, ...
       'MarkerFaceColor', 'w', 'HandleVisibility', 'off');
  plot([k.p(1,1) k.p(1,2)], [0 k.p(3,2)], ':', 'Color', [0.4 0.4 0.4], ...
       'HandleVisibility', 'off');
end

function v = env_at(centres, env, r)
% Envelope torque in the last reach bin at or below r.
  i = find(centres <= r & all(~isnan(env), 2)', 1, 'last');
  v = env(i,:);
end

function R = rpy2r(rpy)
% URDF roll-pitch-yaw to rotation matrix: R = Rz(yaw)*Ry(pitch)*Rx(roll)
  R = axang([0 0 1], rpy(3)) * axang([0 1 0], rpy(2)) * axang([1 0 0], rpy(1));
end

function R = axang(u, th)
% Rotation by angle th (rad) about unit axis u (Rodrigues).
  u = u(:) / norm(u);
  K = [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0];
  R = eye(3) + sin(th)*K + (1 - cos(th))*(K*K);
end

function yline_(y, style, label)
% Horizontal reference line (works without the R2018b yline function).
  xl = xlim;
  plot(xl, [y y], style, 'LineWidth', 1.2, 'HandleVisibility', 'off');
  text(xl(1) + 0.25*diff(xl), y, label, 'HorizontalAlignment', 'center', ...
       'VerticalAlignment', 'bottom', 'FontSize', 8);
end

function xline_(x, style, label)
% Vertical reference line (works without the R2018b xline function).
  yl = ylim;
  plot([x x], yl, style, 'LineWidth', 1.2, 'HandleVisibility', 'off');
  text(x, yl(2), [' ' label], 'VerticalAlignment', 'top', 'FontSize', 8, ...
       'Rotation', 90, 'HorizontalAlignment', 'right');
end

function save_fig(f, name)
% Save a figure as a 300 dpi PNG.
  if exist('exportgraphics', 'file')
    exportgraphics(f, name, 'Resolution', 300);
  else
    print(f, name, '-dpng', '-r300');
  end
end
