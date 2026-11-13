%% KUKA reachability study — drain-cleaning robot (MREN 403)
% Can the iiwa 7 on the Ewellix lift hold the continuum robot pointing
% straight down over a sink drain and insert it, for the range of sinks in
% scope?  Tests two strategies:
%   Mode A: lift stays fixed, the ARM does the insertion stroke
%   Mode B: arm holds one pose, the LIFT does the insertion stroke
%
% Inputs (in the same folder as this script unless noted):
%   que36.urdf                    full robot model exported from the Ridgeback
%   continuum_sim_parameters.xlsx parameter workbook (Sink + Design Inputs sheets);
%                                 this folder or the repo root one level up
%   que36_pkgs/   (optional)      mesh packages from que36_pkgs.tgz. If present,
%                                 meshes are drawn and used for a full collision check.
%
% Frame: Ridgeback base_link (x forward toward the sink, z up). The floor is
% at z = -0.0759 m (wheel radius from the URDF). All sheet values are in mm.
%
% Requires: Robotics System Toolbox (R2025a tested target).

clear; clc; close all;
rng(1);   % reproducible IK seeds
here = fileparts(mfilename("fullpath"));
if here == "", here = pwd; end

urdfFile  = fullfile(here, "que36.urdf");
% Workbook: same folder as this script, or the repo root one level up
paramFile = fullfile(here, "continuum_sim_parameters.xlsx");
if ~isfile(paramFile), paramFile = fullfile(here, "..", "continuum_sim_parameters.xlsx"); end
meshRoot  = fullfile(here, "que36_pkgs");
outDir    = fullfile(here, "reach_results");
if ~isfolder(outDir), mkdir(outDir); end

nLift  = 11;     % lift positions to test
nDrain = 6;      % drain positions (front edge to drain) to test
nStep  = 11;     % points along the insertion stroke

%% ---------------- Parameters from the workbook -------------------------
S = readParams(paramFile, "Sink");
D = readParams(paramFile, "Design Inputs");
mm = @(P, name, field) getv(P, name, field) / 1000;   % sheet mm -> m

zFloor  = -0.0759;
H_rim   = mm(S,"H_rim","base");   h_bowl = mm(S,"h_bowl","base");
D_sink  = mm(S,"D_sink","base");  W_sink = mm(S,"W_sink","base");
L_bowl  = mm(S,"L_bowl","base");  W_bowl = mm(S,"W_bowl","base");
y_drain = mm(S,"y_drain","base");
f_off   = mm(S,"f_off","base");   f_h    = mm(S,"f_h","base");
wallOn  = upper(string(getv(S,"wall","base"))) == "Y";
x_bump  = mm(S,"x_bump","base");  gap    = mm(S,"gap","base");
L_pack  = mm(S,"L_pack","base");  R_pack = mm(S,"R_pack","base");
r_link  = mm(S,"r_link","base");  clr    = mm(S,"clr","base");
L_cont  = mm(D,"L","base");       r_cont = mm(D,"OD","base")/2;
dDrainRange = [mm(S,"d_drain","min"), mm(S,"d_drain","max")];
dDrainBase  = mm(S,"d_drain","base");

z_rim = zFloor + H_rim;
z_out = z_rim - h_bowl;
% Insertion stroke: stop with the actuator pack one clearance above the outlet
L_ins = L_cont - clr;
fprintf("Outlet %.0f mm above floor; insertion stroke %.0f mm\n", (z_out-zFloor)*1000, L_ins*1000);

targetDepth = readQuickCheck(paramFile, "Target tip insertion depth");
if ~isnan(targetDepth) && L_ins*1000 < targetDepth
    warning("Stroke %.0f mm is shorter than the plug target depth %.0f mm: " + ...
        "the actuator pack would hit the bowl before the plug is in place. " + ...
        "The continuum needs to be longer than L = %.0f mm.", L_ins*1000, targetDepth, L_cont*1000);
end

%% ---------------- Robot model -----------------------------------------
hasMeshes = isfolder(meshRoot);
if hasMeshes
    % Rewrite package://<pkg>/ mesh paths to absolute paths in a temporary
    % copy of the URDF (more reliable than importrobot's MeshPath option).
    pk = dir(fullfile(meshRoot, "**", "package.xml"));
    txt = fileread(urdfFile);
    for i = 1:numel(pk)
        [~, pkgName] = fileparts(pk(i).folder);
        absDir = strrep(pk(i).folder, "\", "/");
        txt = char(strrep(txt, "package://" + pkgName + "/", absDir + "/"));
    end
    nLeft = numel(strfind(txt, "package://"));
    tmpUrdf = fullfile(tempdir, "que36_resolved.urdf");
    fid = fopen(tmpUrdf, "w", "n", "UTF-8"); fprintf(fid, "%s", txt); fclose(fid);
    robot = importrobot(tmpUrdf);
    fprintf("Loaded robot with meshes from %d packages (%d mesh references unresolved).\n", numel(pk), nLeft);
else
    warning("off", "robotics:robotmanip:urdfimporter:FileNotFound");
    robot = importrobot(urdfFile);
    fprintf("Loaded robot WITHOUT meshes (kinematics only; approximate collision check).\n");
end
robot.DataFormat = "row";
jnFull  = configJointNames(robot);
iLift   = find(jnFull == "ewellix_lift_top_joint");
iArmFul = arrayfun(@(k) find(jnFull == "iiwa_joint_" + k), 1:7);
liftLim = getBody(robot, 'ewellix_lift_top_link').Joint.PositionLimits;

% Arm-only tree with iiwa_link_0 as its base (copied body by body)
arm = rigidBodyTree("DataFormat", "row");
arm.BaseName = 'iiwa_link_0';
chainNames = ["iiwa_link_1","iiwa_link_2","iiwa_link_3","iiwa_link_4", ...
              "iiwa_link_5","iiwa_link_6","iiwa_link_7","iiwa_link_ee"];
parentName = "iiwa_link_0";
for n = chainNames
    addBody(arm, copy(getBody(robot, char(n))), char(parentName));
    parentName = n;
end
jnArm = configJointNames(arm);
armLim = zeros(7,2);
for k = 1:7
    b = findBodyByJoint(arm, "iiwa_joint_" + k);
    armLim(k,:) = b.Joint.PositionLimits;
end
ik = inverseKinematics("RigidBodyTree", arm);
ik.SolverParameters.MaxIterations = 300;
ikW = [1 1 0 1 1 1];        % free rotation about the tool axis

linkNames = ["iiwa_link_1","iiwa_link_2","iiwa_link_3","iiwa_link_4", ...
             "iiwa_link_5","iiwa_link_6","iiwa_link_7","iiwa_link_ee"];

T_link0 = @(lift) getTransform(robot, setq(homeConfiguration(robot), iLift, lift), 'iiwa_link_0');
Rdown = [1 0 0; 0 -1 0; 0 0 -1];   % tool z-axis pointing straight down

%% ---------------- Sweep: Mode A (arm does the insertion) ---------------
liftVals  = linspace(liftLim(1), liftLim(2), nLift);
drainVals = linspace(dDrainRange(1), dDrainRange(2), nDrain);
statusA = zeros(nDrain, nLift);     % 0 ok, 1 unreachable, 2 collision, 3 joint flip
marginA = nan(nDrain, nLift);       % min joint-limit margin along stroke [deg]
pathA   = cell(nDrain, nLift);

for a = 1:nDrain
    env = buildSink(x_bump+gap, drainVals(a), y_drain, D_sink, W_sink, L_bowl, W_bowl, ...
                    z_out, z_rim, f_off, f_h, wallOn);
    for b = 1:nLift
        TB = T_link0(liftVals(b));
        [st, mg, qs] = runStroke(arm, ik, ikW, TB, env, x_bump+gap+drainVals(a), y_drain, ...
            z_out, L_cont, L_ins, L_pack, R_pack, r_cont, r_link, clr, nStep, armLim, linkNames, Rdown, 0);
        statusA(a,b) = st; marginA(a,b) = mg; pathA{a,b} = qs;
    end
    fprintf("Mode A  d_drain = %3.0f mm : %s\n", drainVals(a)*1000, strjoin(string(statusA(a,:)), " "));
end

%% ---------------- Mode B (lift does the insertion) ---------------------
% Arm pose fixed; lift starts high enough to cover the whole stroke.
statusB = nan(1, nDrain); marginB = nan(1, nDrain); qB = cell(1, nDrain);
liftStart = liftLim(2);
if liftLim(2) - liftLim(1) < L_ins
    warning("Lift travel (%.0f mm) is shorter than the stroke (%.0f mm): Mode B not possible alone.", ...
        diff(liftLim)*1000, L_ins*1000);
else
    for a = 1:nDrain
        env = buildSink(x_bump+gap, drainVals(a), y_drain, D_sink, W_sink, L_bowl, W_bowl, ...
                        z_out, z_rim, f_off, f_h, wallOn);
        liftPath = linspace(liftStart, liftStart - L_ins, nStep);
        [st, mg, qs] = runStroke(arm, ik, ikW, T_link0(liftStart), env, x_bump+gap+drainVals(a), y_drain, ...
            z_out, L_cont, L_ins, L_pack, R_pack, r_cont, r_link, clr, nStep, armLim, linkNames, Rdown, ...
            liftPath - liftStart);
        statusB(a) = st; marginB(a) = mg; qB{a} = qs;
    end
end
fprintf("Mode B  status by d_drain: %s\n", strjoin(string(statusB), " "));

%% ---------------- Optional full-mesh collision check -------------------
meshNote = "approximate (link capsules)";
if hasMeshes
    try
        nHit = 0;
        for a = 1:nDrain
            env = buildSink(x_bump+gap, drainVals(a), y_drain, D_sink, W_sink, L_bowl, W_bowl, ...
                            z_out, z_rim, f_off, f_h, wallOn);
            objs = cellfun(@(B) boxObj(B), env, "UniformOutput", false);
            for b = 1:nLift
                if statusA(a,b) ~= 0, continue; end
                for k = 1:size(pathA{a,b},1)
                    cfg = setq(homeConfiguration(robot), iLift, liftVals(b));
                    cfg(iArmFul) = pathA{a,b}(k,:);
                    if any(checkCollision(robot, cfg, objs, "IgnoreSelfCollision", "on"))
                        statusA(a,b) = 2; marginA(a,b) = NaN; nHit = nHit + 1; break;
                    end
                end
            end
        end
        meshNote = sprintf("full meshes (%d extra collisions found)", nHit);
    catch err
        warning("Mesh collision check skipped: %s", err.message);
    end
end

%% ---------------- Plots ------------------------------------------------
f1 = figure("Name","Mode A","Position",[100 100 820 480]);
img = marginA; imagesc(liftVals*1000, drainVals*1000, img, "AlphaData", ~isnan(img));
set(gca, "YDir", "normal", "Color", [0.85 0.85 0.85]); colormap(parula); cb = colorbar;
cb.Label.String = "min joint-limit margin along stroke [deg]";
hold on
codes = ["", "IK", "C", "flip"];
for a = 1:nDrain
    for b = 1:nLift
        if statusA(a,b) > 0
            text(liftVals(b)*1000, drainVals(a)*1000, codes(statusA(a,b)+1), ...
                "HorizontalAlignment","center", "FontSize", 8);
        end
    end
end
xlabel("lift position [mm] (0 = KUKA base lowest)"); ylabel("sink front edge to drain [mm]");
title({"Mode A: arm does the insertion (lift fixed)", ...
       sprintf("rim %.0f mm, bowl %.0f mm, gap %.0f mm, collision: %s", H_rim*1000, h_bowl*1000, gap*1000, meshNote)});
exportgraphics(f1, fullfile(outDir, "modeA_map.png"));

f2 = figure("Name","Mode B","Position",[150 150 700 420]);
okB = statusB == 0;
bar(drainVals*1000, marginB .* okB); hold on
for a = find(~okB)
    text(drainVals(a)*1000, 2, codes(statusB(a)+1), "HorizontalAlignment","center");
end
xlabel("sink front edge to drain [mm]"); ylabel("min joint-limit margin [deg]");
title("Mode B: lift does the insertion (arm pose fixed)"); grid on
exportgraphics(f2, fullfile(outDir, "modeB_margin.png"));

% ---- Snapshots and animations at the baseline drain position ----------
[~, aBase] = min(abs(drainVals - dDrainBase));
envBase = buildSink(x_bump+gap, drainVals(aBase), y_drain, D_sink, W_sink, L_bowl, W_bowl, ...
                    z_out, z_rim, f_off, f_h, wallOn);
sVals = linspace(0, L_ins, nStep);
viz = struct("robot", robot, "iLift", iLift, "iArm", iArmFul, "env", {envBase}, ...
             "z_out", z_out, "L_cont", L_cont, "L_pack", L_pack, "sVals", sVals, "outDir", outDir);
% Mode B: arm frozen, lift lowers from the top of its travel
if statusB(aBase) == 0
    liftPathB = liftStart - sVals;
    visualizeStroke(viz, liftPathB, repmat(qB{aBase}(1,:), nStep, 1), ...
        sprintf("Mode B (lift inserts), drain %.0f mm back", drainVals(aBase)*1000), "modeB");
end
% Mode A: best lift setting for this drain, arm does the stroke
okA = statusA(aBase,:) == 0;
if any(okA)
    mA = marginA(aBase,:); mA(~okA) = -Inf; [~, bBest] = max(mA);
    visualizeStroke(viz, repmat(liftVals(bBest), 1, nStep), pathA{aBase,bBest}, ...
        sprintf("Mode A (arm inserts), drain %.0f mm back, lift %.0f mm", drainVals(aBase)*1000, liftVals(bBest)*1000), "modeA");
end

save(fullfile(outDir, "reach_results.mat"), "liftVals","drainVals","statusA","marginA","statusB","marginB","pathA","qB");
fprintf("\nStatus codes: 0 ok, 1 unreachable (IK), 2 collision, 3 joint flip mid-stroke\n");
fprintf("Results saved to %s\n", outDir);

%% ======================= Local functions ===============================
function [status, margin, qs] = runStroke(arm, ik, w, TB, env, xd, yd, z_out, L_cont, L_ins, ...
        L_pack, R_pack, r_cont, r_link, clr, nStep, lim, linkNames, Rdown, liftOffsets)
% Mode A: liftOffsets = 0  -> lift fixed, arm tracks the stroke (IK at every point)
% Mode B: liftOffsets = vector -> arm frozen in its start pose, lift lowers everything
% The arm is redundant (7 joints, 6-DOF pose with free tool roll), so many
% start poses exist. We collect several, keep the ones farthest from joint
% limits, follow the stroke from each, and return the best complete stroke.
    s   = linspace(0, L_ins, nStep);
    tgt = @(k) [Rdown [xd; yd; z_out + (L_cont - s(k)) + L_pack]; 0 0 0 1];
    starts = solveIKAll(arm, ik, w, TB \ tgt(1), lim, 30);
    status = 1; margin = NaN; qs = [];
    if isempty(starts), return; end
    starts = starts(1:min(5, end), :);
    bestMargin = -Inf;
    for c = 1:size(starts, 1)
        [st, mg, Q] = followStroke(starts(c,:), arm, ik, w, TB, tgt, env, z_out, L_pack, R_pack, ...
                                   r_cont, r_link, clr, nStep, lim, linkNames, liftOffsets);
        if st == 0 && mg > bestMargin
            bestMargin = mg; status = 0; margin = mg; qs = Q;
        elseif status ~= 0 && c == 1
            status = st;                      % report why the best start failed
        end
    end
end

function [status, margin, qs] = followStroke(q1, arm, ik, w, TB, tgt, env, z_out, L_pack, R_pack, ...
                                             r_cont, r_link, clr, nStep, lim, linkNames, liftOffsets)
    modeB = numel(liftOffsets) > 1;
    status = 0; margin = NaN; qs = zeros(nStep, 7); qPrev = q1;
    for k = 1:nStep
        if k == 1
            q = q1; TBk = TB;
        elseif modeB
            q = q1; TBk = trvec2tform([0 0 liftOffsets(k)]) * TB;
        else
            TBk = TB;
            [q, ok] = solveIK(arm, ik, w, TB \ tgt(k), qPrev);
            if ~ok, status = 1; return; end
            if max(abs(q - qPrev)) > 0.6, status = 3; return; end
        end
        Tk = tgt(k);
        if collides(arm, q, TBk, env, linkNames, Tk(1:3,4), z_out, L_pack, R_pack, r_cont, r_link, clr)
            status = 2; return;
        end
        qs(k,:) = q; qPrev = q;
    end
    margin = min(jointMargin(qs, lim)) * 180/pi;
end

function Qs = solveIKAll(arm, ik, w, Tl, lim, nRand)
% All distinct valid IK solutions from many seeds, sorted by joint-limit margin.
    seeds = [0 0.8 0 -1.4 0 0.9 0; 0 -0.8 0 1.4 0 -0.9 0; zeros(1,7)];
    for r = 1:nRand, seeds(end+1,:) = randomConfiguration(arm); end %#ok<AGROW>
    Qs = zeros(0, 7);
    for r = 1:size(seeds,1)
        [qc, ok] = ikOnce(arm, ik, w, Tl, seeds(r,:));
        if ok && (isempty(Qs) || min(max(abs(Qs - qc), [], 2)) > 0.05)
            Qs(end+1,:) = qc; %#ok<AGROW>
        end
    end
    if ~isempty(Qs)
        [~, order] = sort(jointMargin(Qs, lim), "descend");
        Qs = Qs(order, :);
    end
end

function [q, ok] = solveIK(arm, ik, w, Tl, qPrev)
% Warm-started IK along the stroke: previous pose first, then small perturbations.
    seeds = [qPrev; qPrev + 0.05*randn(3, 7)];
    for r = 1:size(seeds,1)
        [q, ok] = ikOnce(arm, ik, w, Tl, seeds(r,:));
        if ok, return; end
    end
end

function [q, ok] = ikOnce(arm, ik, w, Tl, seed)
    [q, ~] = ik('iiwa_link_ee', Tl, w, seed);
    Ta = getTransform(arm, q, 'iiwa_link_ee');
    ePos = norm(Ta(1:3,4) - Tl(1:3,4));
    eAng = acosd(max(-1, min(1, dot(Ta(1:3,3), Tl(1:3,3)))));
    ok = ePos < 0.002 && eAng < 1;
end

function m = jointMargin(Q, lim)
% Smallest distance to any joint limit for each row of Q [rad].
    m = min(min(Q - lim(:,1)', lim(:,2)' - Q), [], 2);
end

function hit = collides(arm, q, TB, env, linkNames, pTool, z_out, L_pack, R_pack, r_cont, r_link, clr)
% Approximate collision check: arm as capsules between link origins,
% actuator pack as a capsule, exposed continuum as a thin capsule.
    P = zeros(3, numel(linkNames));
    for i = 1:numel(linkNames)
        T = TB * getTransform(arm, q, char(linkNames(i)));
        P(:,i) = T(1:3,4);
    end
    pts = []; rad = [];
    for i = 1:size(P,2)-1
        seg = P(:,i) + (P(:,i+1) - P(:,i)) * linspace(0,1,8);
        pts = [pts seg]; rad = [rad repmat(r_link, 1, 8)]; %#ok<AGROW>
    end
    pPackEnd = pTool - [0;0;L_pack];
    % Pack as spheres whose lowest point sits at the pack's bottom face
    pts = [pts, pTool + ((pPackEnd + [0;0;R_pack + 1e-3]) - pTool) * linspace(0,1,6)];
    rad = [rad, repmat(R_pack, 1, 6)];
    % Exposed continuum above the outlet. It enters the drain by design, so
    % stop checking just above the outlet rather than flagging the bowl floor.
    zStop = z_out + r_cont + clr + 0.005;
    if pPackEnd(3) > zStop
        zc = linspace(pPackEnd(3), zStop, 8);
        pts = [pts, [repmat(pPackEnd(1:2), 1, 8); zc]];
        rad = [rad, repmat(r_cont, 1, 8)];
    end
    hit = false;
    for k = 1:numel(env)
        B = env{k};
        dvec = max(max(B.lo(:) - pts, 0), pts - B.hi(:));
        if any(vecnorm(dvec) < rad + clr), hit = true; return; end
    end
end

function env = buildSink(x_front, d_drain, yd, D_sink, W_sink, L_bowl, W_bowl, z_out, z_rim, f_off, f_h, wallOn)
% Sink as axis-aligned boxes around a rectangular bowl opening centred on the drain.
    xd = x_front + d_drain; xb = x_front + D_sink; zl = z_out - 0.15;
    x1 = max(x_front, xd - L_bowl/2); x2 = min(xb, xd + L_bowl/2);
    env = {};
    env{end+1} = box([x_front -W_sink/2 zl], [x1 W_sink/2 z_rim]);           % front of bowl
    env{end+1} = box([x2 -W_sink/2 zl], [xb W_sink/2 z_rim]);                % back of bowl
    env{end+1} = box([x1 -W_sink/2 zl], [x2 yd-W_bowl/2 z_rim]);             % left
    env{end+1} = box([x1 yd+W_bowl/2 zl], [x2 W_sink/2 z_rim]);              % right
    env{end+1} = box([x1 yd-W_bowl/2 zl], [x2 yd+W_bowl/2 z_out]);           % bowl floor
    env{end+1} = box([xd+f_off yd-0.03 z_rim], [xb yd+0.03 z_rim+f_h]);      % faucet spout
    if wallOn
        env{end+1} = box([xb -2 -0.1], [xb+0.3 2 2.5]);                      % wall
    end
    env = env(cellfun(@(B) all(B.hi > B.lo), env));
end

function B = box(lo, hi), B.lo = lo(:); B.hi = hi(:); end

function obj = boxObj(B)
    sz = (B.hi - B.lo)'; obj = collisionBox(sz(1), sz(2), sz(3));
    obj.Pose = trvec2tform(((B.hi + B.lo)/2)');
end

function drawBox(B, ax, lims)
    lo = max(B.lo, lims(:,1)); hi = min(B.hi, lims(:,2));
    if any(hi <= lo), return; end
    [X,Y,Z] = ndgrid([lo(1) hi(1)], [lo(2) hi(2)], [lo(3) hi(3)]);
    V = [X(:) Y(:) Z(:)];
    F = [1 3 4 2; 5 6 8 7; 1 2 6 5; 3 7 8 4; 1 5 7 3; 2 4 8 6];
    patch(ax, "Vertices", V, "Faces", F, "FaceColor", [0.7 0.75 0.8], "FaceAlpha", 0.3, "EdgeColor", [0.45 0.45 0.45]);
end

function visualizeStroke(viz, liftPath, Q, ttl, tag)
% Start / middle / end snapshots (PNG) and an insertion animation (GIF).
    lims = [-0.6 1.3; -0.7 0.7; -0.08 2.0];
    nStep = numel(liftPath);
    % Snapshots
    f = figure("Position", [80 80 1500 520], "Color", "w");
    tl = tiledlayout(f, 1, 3, "TileSpacing", "compact");
    picks = [1, ceil(nStep/2), nStep]; names = ["Start", "Halfway", "Fully inserted"];
    for i = 1:3
        ax = nexttile(tl);
        drawFrame(ax, viz, liftPath(picks(i)), Q(picks(i),:), viz.sVals(picks(i)), lims, false, []);
        title(ax, sprintf("%s (%.0f mm in)", names(i), viz.sVals(picks(i))*1000));
    end
    title(tl, ttl);
    exportgraphics(f, fullfile(viz.outDir, tag + "_snapshots.png"), "Resolution", 150);
    % Animation: interpolate between stroke points for smooth motion
    nF = 4*(nStep-1) + 1; t = linspace(1, nStep, nF);
    LP = interp1(1:nStep, liftPath(:), t); QP = interp1(1:nStep, Q, t); SP = interp1(1:nStep, viz.sVals(:), t);
    f2 = figure("Position", [100 100 820 680], "Color", "w"); ax = axes(f2);
    gifFile = fullfile(viz.outDir, tag + "_insertion.gif");
    if isfile(gifFile), delete(gifFile); end
    h = []; frames = [1:nF, repmat(nF, 1, 6)];   % hold the last frame briefly
    for i = 1:numel(frames)
        k = frames(i);
        h = drawFrame(ax, viz, LP(k), QP(k,:), SP(k), lims, true, h);
        title(ax, sprintf("%s\ninserted %.0f mm", ttl, SP(k)*1000));
        drawnow;
        exportgraphics(f2, gifFile, "Append", i > 1);
    end
end

function h = drawFrame(ax, viz, lift, q, s, lims, fast, h)
    cfg = homeConfiguration(viz.robot);
    cfg(viz.iLift) = lift; cfg(viz.iArm) = q;
    show(viz.robot, cfg, "Parent", ax, "Frames", "off", "PreservePlot", false, "FastUpdate", fast);
    Tee = getTransform(viz.robot, cfg, 'iiwa_link_ee');
    p0 = Tee(1:3,4); p1 = p0 + Tee(1:3,3) * viz.L_pack;
    p2 = [p1(1); p1(2); viz.z_out];
    if isempty(h)
        hold(ax, "on");
        for k = 1:numel(viz.env), drawBox(viz.env{k}, ax, lims); end
        h.pack = plot3(ax, [p0(1) p1(1)], [p0(2) p1(2)], [p0(3) p1(3)], "-", "LineWidth", 9, "Color", [0.85 0.4 0.1]);
        h.cont = plot3(ax, [p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], "-", "LineWidth", 2.5, "Color", [0.1 0.3 0.85]);
        xlim(ax, lims(1,:)); ylim(ax, lims(2,:)); zlim(ax, lims(3,:));
        axis(ax, "manual"); daspect(ax, [1 1 1]); view(ax, -37.5, 18);
        xlabel(ax, "x [m]"); ylabel(ax, "y [m]"); zlabel(ax, "z [m]");
    else
        set(h.pack, "XData", [p0(1) p1(1)], "YData", [p0(2) p1(2)], "ZData", [p0(3) p1(3)]);
        set(h.cont, "XData", [p1(1) p2(1)], "YData", [p1(2) p2(2)], "ZData", [p1(3) p2(3)]);
    end
    if ~fast, h = []; end
end

function names = configJointNames(rbt)
    names = strings(0);
    for i = 1:rbt.NumBodies
        j = rbt.Bodies{i}.Joint;
        if j.Type ~= "fixed", names(end+1) = string(j.Name); end %#ok<AGROW>
    end
end

function b = findBodyByJoint(rbt, jname)
    for i = 1:rbt.NumBodies
        if string(rbt.Bodies{i}.Joint.Name) == jname, b = rbt.Bodies{i}; return; end
    end
    error("Joint %s not found", jname);
end

function q = setq(q, i, v), q(i) = v; end

function P = readParams(file, sheet)
% Reads Symbol / Baseline / Sweep min / Sweep max columns of a workbook sheet.
    C = readcell(file, "Sheet", sheet);
    P = struct();
    for r = 2:size(C,1)
        sym = C{r,3};
        if ~(ischar(sym) || isstring(sym)), continue; end
        f = matlab.lang.makeValidName(string(sym));
        P.(f) = struct("base", C{r,5}, "min", C{r,6}, "max", C{r,7});
    end
end

function v = getv(P, name, field)
    if ~isfield(P, name), error("Parameter '%s' not found in workbook", name); end
    v = P.(name).(field);
    if ismissing(v), error("Parameter '%s' has no %s value", name, field); end
end

function v = readQuickCheck(file, label)
    v = NaN;
    try
        C = readcell(file, "Sheet", "Quick Checks");
        for r = 1:size(C,1)
            if (ischar(C{r,1}) || isstring(C{r,1})) && string(C{r,1}) == label && isnumeric(C{r,2})
                v = C{r,2}; return;
            end
        end
    catch
    end
end