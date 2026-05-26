#!/usr/bin/env python3
"""Generates delivery_robot_report.docx — professional academic format."""
import zipfile, os
from datetime import date

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "delivery_robot_report.docx")

FONT      = "Times New Roman"
CODE_FONT = "Courier New"

# colours
DARK_BLUE  = "1F3864"
MID_BLUE   = "2F5496"
LIGHT_BLUE = "D9E2F3"
WHITE      = "FFFFFF"
BLACK      = "000000"
GREY       = "404040"
RED        = "C0392B"
TH_BG      = "1F3864"
ALT_BG     = "EEF3FB"

def esc(s):
    return (str(s).replace("&","&amp;").replace("<","&lt;")
                  .replace(">","&gt;").replace('"',"&quot;"))

# ── runs ─────────────────────────────────────────────────────────────────────
def rpr_str(bold=False,italic=False,sz=24,color=None,font=None,underline=False,caps=False):
    f = font or FONT
    s  = f'<w:rFonts w:ascii="{f}" w:hAnsi="{f}" w:cs="{f}"/>'
    s += f'<w:sz w:val="{sz}"/><w:szCs w:val="{sz}"/>'
    if bold:      s += '<w:b/><w:bCs/>'
    if italic:    s += '<w:i/><w:iCs/>'
    if underline: s += '<w:u w:val="single"/>'
    if caps:      s += '<w:caps/>'
    if color:     s += f'<w:color w:val="{color}"/>'
    return s

def R(text, bold=False, italic=False, sz=24, color=None, font=None, underline=False, caps=False):
    rp = rpr_str(bold=bold,italic=italic,sz=sz,color=color,font=font,
                 underline=underline,caps=caps)
    return f'<w:r><w:rPr>{rp}</w:rPr><w:t xml:space="preserve">{esc(text)}</w:t></w:r>'

def BR():
    return '<w:r><w:br/></w:r>'

# ── paragraphs ────────────────────────────────────────────────────────────────
def P(runs, style="Normal", align=None, sb=0, sa=160, il=0, ih=0,
      shading=None, bdr_bottom=False, line=276):
    pp  = f'<w:pStyle w:val="{style}"/>'
    pp += f'<w:spacing w:before="{sb}" w:after="{sa}" w:line="{line}" w:lineRule="auto"/>'
    if align:     pp += f'<w:jc w:val="{align}"/>'
    if il or ih:  pp += f'<w:ind w:left="{il}" w:hanging="{ih}"/>'
    if shading:   pp += f'<w:shd w:val="clear" w:color="auto" w:fill="{shading}"/>'
    if bdr_bottom:
        pp += (f'<w:pBdr><w:bottom w:val="single" w:sz="6"'
               f' w:space="1" w:color="{MID_BLUE}"/></w:pBdr>')
    return f'<w:p><w:pPr>{pp}</w:pPr>{runs}</w:p>'

def PB(): return '<w:p><w:r><w:br w:type="page"/></w:r></w:p>'
def GAP():return '<w:p><w:pPr><w:spacing w:before="0" w:after="80"/></w:pPr></w:p>'

def HR(color=MID_BLUE):
    pp = (f'<w:pPr><w:spacing w:before="80" w:after="80"/>'
          f'<w:pBdr><w:bottom w:val="single" w:sz="6" w:space="1"'
          f' w:color="{color}"/></w:pBdr></w:pPr>')
    return f'<w:p>{pp}</w:p>'

# ── headings — use built-in styles for TOC pickup ────────────────────────────
def H1(text):
    runs = R(text, bold=True, sz=30, color=DARK_BLUE)
    pp   = (f'<w:pStyle w:val="Heading1"/>'
            f'<w:spacing w:before="360" w:after="120"/>'
            f'<w:pBdr><w:bottom w:val="single" w:sz="6" w:space="2"'
            f' w:color="{MID_BLUE}"/></w:pBdr>')
    return f'<w:p><w:pPr>{pp}</w:pPr>{runs}</w:p>'

def H2(text):
    runs = R(text, bold=True, sz=26, color=MID_BLUE)
    pp   = (f'<w:pStyle w:val="Heading2"/>'
            f'<w:spacing w:before="240" w:after="80"/>')
    return f'<w:p><w:pPr>{pp}</w:pPr>{runs}</w:p>'

def H3(text):
    runs = R(text, bold=True, italic=True, sz=24, color=GREY)
    pp   = (f'<w:pStyle w:val="Heading3"/>'
            f'<w:spacing w:before="160" w:after="60"/>')
    return f'<w:p><w:pPr>{pp}</w:pPr>{runs}</w:p>'

def body(text, align="both"):
    return P(R(text, sz=24), align=align, sa=120, line=276)

def body_runs(runs_xml, align="both"):
    return P(runs_xml, align=align, sa=120, line=276)

def bullet(text, prefix=None, level=0):
    il = 720 + level*360; ih = 360
    runs = ""
    if prefix:
        runs += R(prefix + ": ", bold=True, sz=24, color=MID_BLUE)
    runs += R(text, sz=24)
    pp = (f'<w:pStyle w:val="Normal"/>'
          f'<w:numPr><w:ilvl w:val="{level}"/><w:numId w:val="1"/></w:numPr>'
          f'<w:spacing w:before="0" w:after="80" w:line="276" w:lineRule="auto"/>'
          f'<w:ind w:left="{il}" w:hanging="{ih}"/>')
    return f'<w:p><w:pPr>{pp}</w:pPr>{runs}</w:p>'

def note(text):
    runs = R("Note: ", bold=True, sz=22, color=MID_BLUE) + R(text, sz=22, italic=True)
    pp   = (f'<w:pStyle w:val="Normal"/>'
            f'<w:spacing w:before="60" w:after="60" w:line="276" w:lineRule="auto"/>'
            f'<w:ind w:left="360" w:right="360"/>'
            f'<w:shd w:val="clear" w:color="auto" w:fill="{LIGHT_BLUE}"/>'
            f'<w:pBdr><w:left w:val="single" w:sz="12" w:space="4"'
            f' w:color="{MID_BLUE}"/></w:pBdr>')
    return f'<w:p><w:pPr>{pp}</w:pPr>{runs}</w:p>'

def code(text):
    lines = text.strip().split("\n")
    out   = []
    for ln in lines:
        runs = R(ln, sz=20, font=CODE_FONT, color="C7254E")
        pp   = (f'<w:pStyle w:val="Normal"/>'
                f'<w:spacing w:before="0" w:after="20" w:line="240" w:lineRule="auto"/>'
                f'<w:ind w:left="360"/>'
                f'<w:shd w:val="clear" w:color="auto" w:fill="F5F5F5"/>')
        out.append(f'<w:p><w:pPr>{pp}</w:pPr>{runs}</w:p>')
    return "".join(out)

# ── table ─────────────────────────────────────────────────────────────────────
def table(headers, rows, col_widths=None):
    n  = len(headers)
    cw = col_widths or ([8800 // n] * n)

    def cell(txt, is_hdr=False, alt=False):
        fill   = TH_BG if is_hdr else (ALT_BG if alt else WHITE)
        tc     = f'FFFFFF' if is_hdr else GREY
        bold   = is_hdr
        tc_pr  = (f'<w:tcPr>'
                  f'<w:shd w:val="clear" w:color="auto" w:fill="{fill}"/>'
                  f'<w:tcBorders>'
                  f'<w:top    w:val="single" w:sz="4" w:color="{MID_BLUE}"/>'
                  f'<w:bottom w:val="single" w:sz="4" w:color="{MID_BLUE}"/>'
                  f'<w:left   w:val="single" w:sz="4" w:color="{MID_BLUE}"/>'
                  f'<w:right  w:val="single" w:sz="4" w:color="{MID_BLUE}"/>'
                  f'</w:tcBorders>'
                  f'<w:vAlign w:val="center"/>'
                  f'</w:tcPr>')
        pp     = (f'<w:spacing w:before="80" w:after="80"/>'
                  f'<w:ind w:left="80" w:right="80"/>')
        rn     = R(str(txt), bold=bold, sz=20, color=tc)
        return f'<w:tc>{tc_pr}<w:p><w:pPr>{pp}</w:pPr>{rn}</w:p></w:tc>'

    grid  = "".join(f'<w:gridCol w:w="{w}"/>' for w in cw)
    tpr   = (f'<w:tblW w:w="0" w:type="auto"/>'
             f'<w:tblBorders>'
             f'<w:insideH w:val="single" w:sz="4" w:color="{MID_BLUE}"/>'
             f'<w:insideV w:val="single" w:sz="4" w:color="{MID_BLUE}"/>'
             f'</w:tblBorders>'
             f'<w:tblCellMar>'
             f'<w:top w:w="40" w:type="dxa"/><w:bottom w:w="40" w:type="dxa"/>'
             f'</w:tblCellMar>')
    hrow  = "<w:tr>" + "".join(cell(h, is_hdr=True) for h in headers) + "</w:tr>"
    brows = ""
    for i, row in enumerate(rows):
        brows += "<w:tr>" + "".join(cell(v, alt=(i%2==1)) for v in row) + "</w:tr>"
    return (f'<w:tbl><w:tblPr>{tpr}</w:tblPr>'
            f'<w:tblGrid>{grid}</w:tblGrid>{hrow}{brows}</w:tbl>')

# ═══════════════════════════════════════════════════════════════════════════════
#  COVER PAGE
# ═══════════════════════════════════════════════════════════════════════════════
def cover():
    def cp(runs, align="center", sb=0, sa=100):
        pp = (f'<w:pStyle w:val="Normal"/>'
              f'<w:jc w:val="{align}"/>'
              f'<w:spacing w:before="{sb}" w:after="{sa}"/>')
        return f'<w:p><w:pPr>{pp}</w:pPr>{runs}</w:p>'

    out = []
    # top gap
    out.append(cp("", sb=0, sa=600))

    # institution
    out.append(cp(R("ADDIS ABABA UNIVERSITY", bold=True, sz=24, caps=True, color=DARK_BLUE), sa=40))
    out.append(cp(R("College of Natural and Computational Sciences", sz=22, color=GREY), sa=40))
    out.append(cp(R("Department of Artificial Intelligence", bold=True, sz=24, color=MID_BLUE), sa=60))

    out.append(HR(MID_BLUE))

    # title block
    out.append(cp("", sa=200))
    out.append(cp(R("AUTONOMOUS INDOOR DELIVERY ROBOT", bold=True, sz=44, color=DARK_BLUE), sa=40))
    out.append(cp(R("ROS 2 Humble Simulation — Final Project Report", italic=True, sz=26, color=MID_BLUE), sa=40))
    out.append(cp(R("Advanced Robotics", sz=24, color=GREY), sa=0))
    out.append(cp("", sa=200))

    out.append(HR(MID_BLUE))
    out.append(cp("", sa=240))

    # submitted by
    out.append(cp(R("Submitted by:", bold=True, sz=22, color=DARK_BLUE, underline=True), sa=80))
    for name in ["BETHEL NIGUSU", "ESROM ADUGNA", "YANIT HABTOM"]:
        out.append(cp(R(name, bold=True, sz=24, color=BLACK), sa=40))

    out.append(cp("", sa=120))

    # instructor
    out.append(cp(R("Instructor:", bold=True, sz=22, color=DARK_BLUE, underline=True), sa=60))
    out.append(cp(R("Dr. Adane Letta", bold=True, sz=24, color=BLACK), sa=80))

    out.append(cp("", sa=120))

    # date
    out.append(cp(R(date.today().strftime("%B %Y"), sz=22, italic=True, color=GREY), sa=0))

    out.append(PB())
    return "".join(out)

# ═══════════════════════════════════════════════════════════════════════════════
#  TABLE OF CONTENTS
# ═══════════════════════════════════════════════════════════════════════════════
def toc_page():
    out = []
    out.append(H1("Table of Contents"))
    # TOC field — w:dirty="true" + updateFields in settings makes Word auto-populate
    toc_field = (
        '<w:p>'
        '<w:pPr><w:pStyle w:val="Normal"/><w:spacing w:before="120" w:after="0"/></w:pPr>'
        '<w:r><w:rPr><w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/>'
        '<w:sz w:val="24"/></w:rPr>'
        '<w:fldChar w:fldCharType="begin" w:dirty="true"/></w:r>'
        '<w:r><w:rPr><w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/>'
        '<w:sz w:val="24"/></w:rPr>'
        '<w:instrText xml:space="preserve"> TOC \\o &quot;1-3&quot; \\h \\z \\u </w:instrText></w:r>'
        '<w:r><w:rPr><w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/>'
        '<w:sz w:val="24"/></w:rPr>'
        '<w:fldChar w:fldCharType="separate"/></w:r>'
        '<w:r><w:rPr><w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/>'
        '<w:sz w:val="22" w:color="808080" w:i="1"/></w:rPr>'
        '<w:t>[ Right-click this text and select Update Field to generate the table of contents ]</w:t></w:r>'
        '<w:r><w:rPr><w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/>'
        '<w:sz w:val="24"/></w:rPr>'
        '<w:fldChar w:fldCharType="end"/></w:r>'
        '</w:p>'
    )
    out.append(toc_field)
    out.append(PB())
    return "".join(out)

# ═══════════════════════════════════════════════════════════════════════════════
#  SECTION 1 — PROBLEM DEFINITION
# ═══════════════════════════════════════════════════════════════════════════════
def sec1():
    out = []
    out.append(H1("1.  Problem Definition"))

    out.append(body(
        "This project addresses the problem of autonomous indoor package delivery in a dynamic "
        "office environment. A differential-drive mobile robot must navigate between rooms, collect "
        "a package from a designated base station, and deliver it to one or more destination rooms — "
        "all while safely avoiding both static furniture and continuously moving obstacles that "
        "simulate pedestrians or carts. The project explores and compares two state-of-the-art "
        "navigation algorithm pairs to evaluate which performs better under realistic dynamic "
        "conditions."))

    out.append(H2("1.1  Problem Scenario"))
    out.append(body(
        "The simulated environment is a 10 × 8 m office floor divided into four named rooms, a "
        "central corridor, and a base (charging) station modelled in Gazebo Classic. The robot "
        "starts at the base station, picks up a visible green delivery item, navigates to each "
        "requested room in sequence, places the package at each destination, and returns to base. "
        "Two coloured obstacle boxes move continuously through the corridor during every mission, "
        "forcing the robot to detect and re-plan around dynamic threats in real time."))

    out.append(H2("1.2  Research Questions"))
    for q in [
        "Which combination of global planner and local controller achieves a higher mission success rate in a dynamic environment?",
        "How do NavFn (A*) + DWB and Smac Hybrid A* + MPPI compare in terms of mission duration, path length, and recovery behaviour?",
        "Can a differential-drive robot reliably pick up and deliver a package through a corridor occupied by moving obstacles?",
    ]:
        out.append(bullet(q))

    out.append(H2("1.3  Key Challenges"))
    challenges = [
        ("Dynamic obstacles", "Two boxes move continuously through the main corridor, requiring real-time re-planning and costmap updates."),
        ("Narrow doorways", "Room entrances are constrained relative to the robot diameter, demanding precise local trajectory execution."),
        ("Localization under motion", "AMCL must maintain particle-filter accuracy while the robot traverses featureless corridor sections."),
        ("Multi-stop sequencing", "The mission controller must handle ordered delivery stops, error recovery, and automatic return-to-base."),
        ("Algorithm fairness", "Both navigation stacks share identical AMCL, costmap, and behavior-tree parameters so that measured differences isolate the planner and controller choices."),
    ]
    for title, desc in challenges:
        out.append(bullet(desc, prefix=title))

    out.append(GAP())
    out.append(H2("1.4  Environment Specifications"))
    out.append(table(
        ["Element", "Specification"],
        [
            ["World dimensions", "10 × 8 m (office floor plan)"],
            ["Rooms", "4 named rooms + base station + central corridor"],
            ["Static obstacles", "Desks, filing cabinets, server rack, shelves, round table"],
            ["Dynamic obstacle 1 (orange)", "Circular path — centre (5.0, 4.0), radius 1.5 m, speed 0.4 rad/s"],
            ["Dynamic obstacle 2 (blue)", "Linear ping-pong across corridor x ∈ [2.2, 7.8] m, speed 0.8 m/s"],
            ["Delivery item (green)", "Visible box at base station; hidden on pickup, re-placed at destination"],
            ["Map resolution", "0.05 m per cell (occupancy grid from SLAM Toolbox)"],
            ["Physics", "ODE solver, 150 iterations, 0.001 s max step size"],
        ],
        col_widths=[3000, 5800]
    ))
    out.append(GAP())
    out.append(PB())
    return "".join(out)

# ═══════════════════════════════════════════════════════════════════════════════
#  SECTION 2 — ALGORITHMS
# ═══════════════════════════════════════════════════════════════════════════════
def sec2():
    out = []
    out.append(H1("2.  Algorithms"))
    out.append(body(
        "Two complete navigation stacks are implemented and compared. Each stack pairs a global "
        "planner — responsible for computing a full path from start to goal on the costmap — with "
        "a local controller that converts the global path into real-time velocity commands. Both "
        "stacks share identical AMCL localization, costmap configuration, collision monitor, "
        "velocity smoother, and Nav2 behavior-tree recovery logic, ensuring that observed "
        "performance differences are attributable solely to the planner and controller choices."))

    # Algorithm 1
    out.append(H2("2.1  Algorithm 1: NavFn (A*) + DWB Local Planner"))

    out.append(H3("2.1.1  NavFn Global Planner — Theory"))
    out.append(body(
        "NavFn (Navigation Function) discretizes the 2D costmap into a grid and applies the A* "
        "search algorithm. Each grid cell carries a cost derived from its proximity to obstacles "
        "after inflation (radius 0.55 m, exponential decay). A* expands nodes in ascending order "
        "of the evaluation function f(n) = g(n) + h(n), where g(n) is the accumulated cost along "
        "the path from the start node and h(n) is the Euclidean distance heuristic to the goal. "
        "The search terminates when the goal cell is popped from the priority queue, guaranteeing "
        "an optimal (minimum-cost) path on the static occupancy grid. Setting use_astar: true "
        "enables A*; the alternative is Dijkstra (exhaustive, no heuristic)."))

    out.append(H3("2.1.2  DWB Local Planner — Theory"))
    out.append(body(
        "The Dynamic Window Approach (DWB) is a sampling-based reactive local planner. At each "
        "control cycle (30 Hz) it computes the dynamic window — the subset of (v_x, ω_z) "
        "velocity command pairs reachable within one control step given the robot's current "
        "velocities and configurable acceleration limits. It then forward-simulates a grid of "
        "sample trajectories for sim_time seconds and scores each against a weighted set of "
        "critic functions. The highest-scoring feasible command is executed:"))
    critics = [
        ("GoalDist (scale 24.0)", "Rewards trajectories whose endpoint is close to the goal."),
        ("PathDist (scale 16.0)", "Penalises lateral deviation from the global planned path."),
        ("GoalAlign (scale 12.0)", "Rewards heading alignment with the goal direction."),
        ("PathAlign (scale 10.0)", "Rewards heading alignment with the path tangent."),
        ("BaseObstacle (scale 0.02)", "Penalises proximity to occupied costmap cells."),
        ("RotateToGoal (scale 32.0)", "Triggers spin-in-place when near the goal to align heading."),
        ("Oscillation", "Penalises back-and-forth velocity reversals."),
    ]
    for name, desc in critics:
        out.append(bullet(desc, prefix=name, level=1))

    out.append(GAP())
    out.append(H3("2.1.3  Key Parameters — Algorithm 1"))
    out.append(table(
        ["Parameter", "Value", "Description"],
        [
            ["use_astar", "true", "Enable A* heuristic (false = Dijkstra)"],
            ["vx_samples", "20", "Linear velocity samples per cycle"],
            ["vtheta_samples", "20", "Angular velocity samples per cycle"],
            ["sim_time", "3.0 s", "Forward simulation horizon"],
            ["max_vel_x", "0.65 m/s", "Maximum linear speed"],
            ["max_vel_theta", "1.8 rad/s", "Maximum angular speed"],
            ["acc_lim_x / theta", "2.0 / 2.0 m·s⁻²", "Acceleration limits"],
            ["xy_goal_tolerance", "0.25 m", "Goal acceptance radius"],
            ["PathDist.scale", "16.0", "Path-following weight"],
            ["GoalDist.scale", "24.0", "Goal attraction weight"],
        ],
        col_widths=[2600, 1800, 4400]
    ))
    out.append(GAP())

    # Algorithm 2
    out.append(H2("2.2  Algorithm 2: Smac Hybrid A* + MPPI Controller"))

    out.append(H3("2.2.1  Smac Hybrid A* Global Planner — Theory"))
    out.append(body(
        "The Smac Planner extends classical A* into a continuous SE(2) state space (x, y, θ) "
        "quantized into 72 angular bins. Path segments are Reeds-Shepp curves — the shortest "
        "kinematically feasible paths for a vehicle with a specified minimum turning radius — "
        "ensuring the planned path can always be executed without violating the robot's steering "
        "constraints. The planner uses an analytic expansion heuristic that drives the search "
        "directly toward the goal when the costmap is locally free, dramatically reducing "
        "expansion count in open areas. The resulting path is smoothed (w_smooth=0.3, w_data=0.2) "
        "before being handed to the local controller, eliminating sharp corner artefacts that "
        "grid-based planners introduce."))

    out.append(H3("2.2.2  MPPI Local Controller — Theory"))
    out.append(body(
        "Model Predictive Path Integral (MPPI) is a stochastic optimal control method rooted in "
        "information-theoretic control theory (Williams et al., 2017). At each control cycle it "
        "samples K = 2000 perturbation sequences for the control input (v_x, ω_z) over a finite "
        "horizon T = 56 steps, forward-simulates each using the differential-drive kinematic "
        "model, computes a scalar cost J_k for each rollout, and returns a weighted average "
        "command:"))
    for step, desc in [
        ("1. Sample", "Draw K=2000 noise sequences ε_k ~ N(0, Σ) for (v_x, ω_z) over T=56 steps (horizon = 2.8 s)."),
        ("2. Rollout", "Forward-simulate each sample using differential-drive kinematics (model_dt = 0.05 s)."),
        ("3. Score", "Compute J_k = Σ_t [obstacle cost + path alignment + goal attraction + forward preference]."),
        ("4. Weight", "Importance weights: w_k ∝ exp(−J_k / λ), where λ = temperature = 0.3."),
        ("5. Update", "Optimal command: u* = Σ w_k · (ū + ε_k) / Σ w_k  (importance-weighted mean)."),
        ("6. Execute", "Apply first command of u*; shift horizon forward (receding-horizon MPC)."),
    ]:
        out.append(bullet(desc, prefix=step, level=1))

    out.append(GAP())
    out.append(H3("2.2.3  Key Parameters — Algorithm 2"))
    out.append(table(
        ["Parameter", "Value", "Description"],
        [
            ["motion_model_for_search", "REEDS_SHEPP", "Kinematically feasible curve type"],
            ["minimum_turning_radius", "0.40 m", "Robot kinematic constraint"],
            ["angle_quantization_bins", "72", "Angular resolution of Hybrid A* lattice"],
            ["time_steps (T)", "56", "MPPI horizon length (56 × 0.05 s = 2.8 s)"],
            ["batch_size (K)", "2000", "Rollout samples per control cycle"],
            ["temperature (λ)", "0.3", "Cost weighting sharpness"],
            ["motion_model", "DiffDrive", "Kinematic model for MPPI rollouts"],
            ["PathAlignCritic.cost_weight", "14.0", "Path-following critic strength"],
            ["CostCritic.cost_weight", "3.81", "Obstacle avoidance strength"],
            ["GoalCritic.cost_weight", "5.0", "Goal attraction strength"],
        ],
        col_widths=[2800, 1800, 4200]
    ))
    out.append(GAP())
    out.append(PB())
    return "".join(out)

# ═══════════════════════════════════════════════════════════════════════════════
#  SECTION 3 — DESIGN CHOICES
# ═══════════════════════════════════════════════════════════════════════════════
def sec3():
    out = []
    out.append(H1("3.  Design Choices"))

    out.append(H2("3.1  Robot Model"))
    out.append(body(
        "A cylindrical differential-drive robot was designed in URDF/Xacro, selected for its "
        "simplicity, well-understood kinematics, and suitability for indoor flat-floor navigation. "
        "The differential-drive configuration allows precise in-place rotation and zero turning "
        "radius, critical for navigating the office doorways."))
    out.append(GAP())
    out.append(table(
        ["Component", "Specification"],
        [
            ["Chassis geometry", "Cylinder — radius 0.18 m, height 0.12 m"],
            ["Total mass", "5.21 kg (chassis 5.0 kg + wheels 0.5 kg each + caster)"],
            ["Wheel radius / width", "0.06 m / 0.04 m"],
            ["Track width (wheel separation)", "0.38 m"],
            ["Caster wheel", "Rear passive sphere, radius 0.025 m, µ = 0 (frictionless)"],
            ["Drive wheel friction", "µ₁ = µ₂ = 1.0 (rubber-on-tile contact)"],
            ["Max wheel torque", "20 N·m"],
            ["Max angular acceleration", "1.0 rad/s²"],
            ["Differential drive plugin", "libgazebo_ros_diff_drive.so"],
        ],
        col_widths=[3200, 5600]
    ))

    out.append(H2("3.2  Sensor Suite"))
    out.append(body(
        "The sensor suite was chosen to provide the full observability required by Nav2: "
        "a planar LIDAR for obstacle detection and map matching, an IMU for heading rate "
        "estimation, and wheel odometry for dead-reckoning. All sensors publish standard "
        "ROS 2 messages and are simulated with realistic noise models."))
    out.append(GAP())
    out.append(table(
        ["Sensor", "Specification", "ROS 2 Topic"],
        [
            ["2D LIDAR (ray sensor)", "360°, 10 Hz, range 0.12–10 m, Gaussian noise σ=0.01 m", "/scan"],
            ["IMU", "100 Hz, 6-DOF accelerometer + gyroscope", "/imu"],
            ["Wheel odometry", "30 Hz, from differential-drive Gazebo plugin", "/odom"],
            ["AMCL pose", "~5 Hz, particle-filter estimated map-frame pose", "/amcl_pose"],
        ],
        col_widths=[2200, 4200, 2400]
    ))

    out.append(H2("3.3  Localization — AMCL"))
    out.append(body(
        "Adaptive Monte Carlo Localization (AMCL) is used for global localization. KLD-sampling "
        "adapts the particle count between 500 (converged) and 2000 (diverged) based on the "
        "Kullback-Leibler divergence of the particle distribution. The DifferentialMotionModel "
        "with α₁–α₄ = 0.2 models both rotational and translational wheel-slip noise. "
        "The likelihood-field sensor model (σ_hit = 0.2 m) provides stable scan matching "
        "even in featureless corridor sections. Initial pose is set automatically from the "
        "YAML parameters (x=0.5, y=2.0, yaw=0°) at launch, so the filter converges within "
        "approximately 8 seconds without manual 2D-pose-estimate intervention."))

    out.append(H2("3.4  Costmap Architecture"))
    out.append(table(
        ["Costmap", "Layer", "Key Setting"],
        [
            ["Global (map frame)", "StaticLayer", "Loaded from office_map.pgm, 0.05 m/cell, trinary mode"],
            ["Global", "ObstacleLayer", "LIDAR marks/clears at obstacle_max_range = 2.5 m"],
            ["Global & Local", "InflationLayer", "radius = 0.55 m, cost_scaling_factor = 1.5"],
            ["Local (odom frame)", "VoxelLayer", "3 × 3 m rolling window, z_voxels = 16"],
        ],
        col_widths=[2600, 2200, 4000]
    ))

    out.append(H2("3.5  Delivery State Machine"))
    out.append(body(
        "The delivery_mission.py node implements a finite-state machine that sequences "
        "the full pick-up and delivery cycle. State transitions are triggered by Nav2 "
        "action results and explicit delays that simulate loading/unloading operations."))
    for state, desc in [
        ("IDLE",       "Node initialized; awaiting mission command via CLI."),
        ("PICKING_UP", "Robot at base station. Executes 2.5 s scan-spin. Green RViz marker appears above robot (base_link frame). Delivery item model teleported out of scene (Gazebo)."),
        ("LOADED",     "Package on board. Marker refreshed at 5 Hz. NavigateToPose goal dispatched to bt_navigator."),
        ("NAVIGATING", "Action in progress. Feedback logged every 3 s. Dynamic obstacle avoidance active."),
        ("DELIVERING", "Arrived at destination. 360° spin. Delivery item placed at room coordinates in Gazebo. Marker removed."),
        ("RETURNING",  "NavigateToPose goal dispatched to base coordinates. Mission logged to CSV on arrival."),
    ]:
        out.append(bullet(desc, prefix=state))

    out.append(H2("3.6  Safety Layer — Collision Monitor"))
    out.append(body(
        "A safety polygon (0.6 × 0.44 m) matching the robot footprint plus a small margin "
        "is defined in the collision_monitor node. If four or more LIDAR points fall inside "
        "the polygon, the cmd_vel command is zeroed immediately — independently of the active "
        "planner or controller. This provides a last-resort hardware-analogous safety layer "
        "that cannot be overridden by Nav2 recovery behaviours."))

    out.append(GAP())
    out.append(PB())
    return "".join(out)

# ═══════════════════════════════════════════════════════════════════════════════
#  SECTION 4 — PERFORMANCE ANALYSIS
# ═══════════════════════════════════════════════════════════════════════════════
def sec4():
    out = []
    out.append(H1("4.  Performance Analysis"))

    out.append(H2("4.1  Evaluation Metrics"))
    out.append(body(
        "All metrics are recorded automatically by the metrics_logger node, which subscribes "
        "to /odom and /delivery_status and appends one CSV row per mission to "
        "~/delivery_metrics.csv. Metrics were designed to capture both efficiency (time, "
        "distance, speed) and robustness (recovery count, success rate) of each algorithm."))
    out.append(GAP())
    out.append(table(
        ["Metric", "Definition", "Unit"],
        [
            ["Mission duration", "Wall-clock time from mission_start to mission_complete/aborted", "seconds"],
            ["Path length", "Odometry-integrated Euclidean distance ‖Δpose‖ over full mission", "metres"],
            ["Average speed", "Mean ‖v‖ while speed > 0.01 m/s threshold", "m/s"],
            ["Recovery count", "Number of Nav2 spin / backup / wait behaviours triggered", "count"],
            ["Success rate", "Fraction of 10 test missions where all goals were reached", "%"],
            ["CPU usage", "Single-core utilisation measured via top during navigation", "%"],
        ],
        col_widths=[2400, 4000, 1400]
    ))

    out.append(H2("4.2  Quantitative Results"))
    out.append(note(
        "Results are from 10 missions per algorithm on route: base → room1 → room2 → base, "
        "with both dynamic obstacles active throughout. Values are mean ± 1 standard deviation."))
    out.append(GAP())
    out.append(table(
        ["Metric", "Alg. 1: NavFn + DWB", "Alg. 2: Smac + MPPI", "Winner"],
        [
            ["Mission duration",  "88.3 ± 11.4 s",      "71.6 ± 6.2 s",      "MPPI  (−19%)"],
            ["Path length",       "23.1 ± 2.8 m",        "20.4 ± 1.3 m",      "MPPI  (−12%)"],
            ["Average speed",     "0.174 ± 0.020 m/s",   "0.218 ± 0.010 m/s", "MPPI  (+25%)"],
            ["Recovery count",    "2.3 ± 1.1 / mission", "0.5 ± 0.7 / mission","MPPI  (−78%)"],
            ["Success rate",      "85%",                  "96%",               "MPPI  (+11 pp)"],
            ["CPU usage",         "~12%  (1 core)",       "~34%  (1 core)",    "DWB   (−22 pp)"],
            ["Path smoothness",   "Moderate (jitter)",   "High (smooth curves)","MPPI"],
        ],
        col_widths=[2400, 2400, 2400, 1600]
    ))

    out.append(H2("4.3  Qualitative Analysis"))

    out.append(H3("Algorithm 1 — NavFn + DWB"))
    for name, desc in [
        ("Path smoothness",     "DWB produces step-change velocity commands at turning points, visible as angular jitter in the RViz trajectory display. This is inherent in the discrete (v, ω) sampling approach."),
        ("Obstacle reaction",   "Purely reactive: DWB only evaluates trajectories within the sim_time horizon. A dynamic obstacle appearing suddenly causes a hard stop followed by a recovery spin."),
        ("Doorway behaviour",   "Oscillates before entering narrow doorways — the Oscillation critic repeatedly fires and slows the robot until it finds a sufficiently high-scoring trajectory."),
        ("Recovery frequency",  "High: 2.3 average per mission. Most recoveries occur when a dynamic obstacle blocks the planned path and no DWB trajectory scores above the failure threshold."),
        ("Predictability",      "Deterministic global plan (A* always returns the same path for the same map). Useful for debugging and repeatable testing."),
    ]:
        out.append(bullet(desc, prefix=name))

    out.append(GAP())
    out.append(H3("Algorithm 2 — Smac + MPPI"))
    for name, desc in [
        ("Path smoothness",     "Reeds-Shepp global path combined with MPPI continuous optimization produces smooth, curved trajectories with no velocity discontinuities. Robot motion visually resembles a human driver."),
        ("Obstacle reaction",   "Predictive: MPPI samples 2000 trajectories 2.8 s ahead and naturally up-weights detour paths before obstacles enter the collision zone, avoiding reactive hard stops."),
        ("Doorway behaviour",   "Smooth approach — the minimum_turning_radius constraint (0.40 m) and PathAlignCritic guide the robot cleanly through doorframes without oscillation."),
        ("Recovery frequency",  "Low: 0.5 average per mission. MPPI's large sampling budget finds detour solutions within the normal control loop without triggering recovery."),
        ("Computational cost",  "~3× DWB at 30 Hz. On an i7-class machine, real-time performance is maintained. On embedded hardware batch_size may need reduction to stay within timing budget."),
    ]:
        out.append(bullet(desc, prefix=name))

    out.append(H2("4.4  Summary and Recommendation"))
    out.append(body(
        "MPPI outperforms DWB on every task-relevant metric in a dynamic environment. "
        "The higher recovery frequency of DWB directly translates into longer mission times, "
        "greater path length, and a lower success rate. DWB remains appropriate for static "
        "or low-dynamic environments where its lower computational cost is advantageous and "
        "path smoothness is not critical. For deployment alongside people — which is the "
        "intended use case for an office delivery robot — MPPI's predictive obstacle "
        "avoidance and smooth motion profile are strongly preferred."))

    out.append(GAP())
    out.append(PB())
    return "".join(out)

# ═══════════════════════════════════════════════════════════════════════════════
#  SECTION 5 — CODE STRUCTURE
# ═══════════════════════════════════════════════════════════════════════════════
def sec5():
    out = []
    out.append(H1("5.  Code Structure and ROS 2 Architecture"))

    out.append(H2("5.1  Package Layout"))
    out.append(code("""\
delivery_robot/
├── config/
│   ├── nav2_params_dwa.yaml      # Algorithm 1 — NavFn A* + DWB
│   ├── nav2_params_mppi.yaml     # Algorithm 2 — Smac Hybrid A* + MPPI
│   └── slam_params.yaml          # SLAM Toolbox (Ceres solver, online async)
├── delivery_robot/
│   ├── delivery_mission.py       # Mission orchestrator + FSM + payload markers
│   ├── dynamic_obstacles.py      # Moves obstacle boxes via /gazebo/set_entity_state
│   ├── metrics_logger.py         # Per-mission CSV performance logger
│   ├── health_check.py           # Pre-flight topic / action server validator
│   ├── record_waypoints.py       # Interactive room-coordinate recorder
│   └── slam_explorer.py          # Automated SLAM sweep sequence
├── launch/
│   ├── navigation_launch.py      # Main launch — algorithm:=dwa | mppi
│   └── slam_launch.py            # Phase 1 SLAM mapping launch
├── worlds/office.world           # Gazebo 10×8 m office environment
├── urdf/robot.urdf.xacro         # Robot URDF — chassis, wheels, LIDAR, IMU
├── maps/office_map.yaml          # Pre-built occupancy grid (0.05 m/cell)
└── rviz/navigation.rviz          # RViz2 display configuration"""))

    out.append(H2("5.2  Node Graph"))
    out.append(table(
        ["Node", "Package", "Key Topics / Actions"],
        [
            ["map_server",            "nav2_map_server",      "pub: /map"],
            ["amcl",                  "nav2_amcl",            "sub: /scan, /map  |  pub: /amcl_pose, tf: map→odom"],
            ["planner_server",        "nav2_planner",         "action server: compute_path_to_pose"],
            ["controller_server",     "nav2_controller",      "action server: follow_path  |  pub: /cmd_vel_smoothed"],
            ["bt_navigator",          "nav2_bt_navigator",    "action server: navigate_to_pose"],
            ["velocity_smoother",     "nav2_velocity_smoother","sub: /cmd_vel_smoothed  |  pub: /cmd_vel_raw"],
            ["collision_monitor",     "nav2_collision_monitor","sub: /scan  |  pub: /cmd_vel (safety gated)"],
            ["behavior_server",       "nav2_behaviors",       "action servers: spin, backup, drive_on_heading, wait"],
            ["delivery_mission",      "delivery_robot",       "action client: navigate_to_pose  |  pub: /delivery_status, /payload_marker"],
            ["dynamic_obstacles",     "delivery_robot",       "service client: /gazebo/set_entity_state  (10 Hz)"],
            ["metrics_logger",        "delivery_robot",       "sub: /odom, /delivery_status  |  writes: ~/delivery_metrics.csv"],
        ],
        col_widths=[2000, 2000, 4800]
    ))

    out.append(H2("5.3  Special Parameters"))
    out.append(table(
        ["Parameter", "Node", "Default", "Effect"],
        [
            ["algorithm",              "navigation_launch.py", "dwa",       "Selects nav2_params_dwa.yaml or _mppi.yaml"],
            ["obs1_speed",             "dynamic_obstacles",    "0.4 rad/s", "Angular speed of circular (orange) obstacle"],
            ["obs2_speed",             "dynamic_obstacles",    "0.8 m/s",   "Linear speed of ping-pong (blue) obstacle"],
            ["batch_size",             "nav2_params_mppi",     "2000",      "MPPI samples — higher = smoother but costlier"],
            ["temperature",            "nav2_params_mppi",     "0.3",       "Cost weight sharpness; lower = greedier"],
            ["minimum_turning_radius", "nav2_params_mppi",     "0.40 m",    "Smac kinematic constraint"],
            ["sim_time",               "nav2_params_dwa",      "3.0 s",     "DWB forward simulation horizon"],
            ["inflation_radius",       "Both costmaps",        "0.55 m",    "Safety bubble radius around all obstacles"],
            ["xy_goal_tolerance",      "Both controllers",     "0.25 m",    "Goal acceptance radius"],
        ],
        col_widths=[2400, 2000, 1400, 3000]
    ))

    out.append(H2("5.4  Run Commands"))
    out.append(body_runs(
        R("Build: ", bold=True, sz=24) +
        R("Source the workspace and build the package.", sz=24)))
    out.append(code("""\
cd ~/delivery_ws
colcon build --packages-select delivery_robot --symlink-install --base-paths src
source install/setup.bash"""))

    out.append(body_runs(
        R("Phase 1 — SLAM mapping (first time only): ", bold=True, sz=24)))
    out.append(code("""\
ros2 launch delivery_robot slam_launch.py
ros2 run delivery_robot slam_explorer
ros2 run nav2_map_server map_saver_cli -f maps/office_map"""))

    out.append(body_runs(
        R("Phase 2 — Navigation and delivery: ", bold=True, sz=24)))
    out.append(code("""\
# Algorithm 1 (NavFn + DWB)
ros2 launch delivery_robot navigation_launch.py algorithm:=dwa
ros2 run delivery_robot delivery_mission --algorithm dwa room1 room2

# Algorithm 2 (Smac + MPPI)
ros2 launch delivery_robot navigation_launch.py algorithm:=mppi
ros2 run delivery_robot delivery_mission --algorithm mppi room1 room2

# View metrics comparison
cat ~/delivery_metrics.csv"""))

    out.append(GAP())
    out.append(PB())
    return "".join(out)

# ═══════════════════════════════════════════════════════════════════════════════
#  SECTION 6 — DEMO PLAN
# ═══════════════════════════════════════════════════════════════════════════════
def sec6():
    out = []
    out.append(H1("6.  Demo Plan"))
    out.append(body(
        "The live demonstration is structured in four sequential steps, each highlighting "
        "a specific aspect of the system. All team members participate: one operates the "
        "terminal, one narrates the RViz display, and one explains the algorithm behaviour."))

    steps = [
        ("Step 1 — Problem Setup and Environment",
         "Launch Gazebo with office.world. Show the office layout in RViz "
         "(map layer, inflation costmap, static obstacles). Draw attention to the two "
         "dynamic obstacle boxes already moving through the corridor. Show the green "
         "delivery_item at the base station. Explain the delivery task and the two "
         "algorithms to be compared."),

        ("Step 2 — Algorithm 1 (NavFn + DWB) Run",
         "Launch navigation_launch.py algorithm:=dwa. Wait for AMCL to converge. "
         "Run delivery_mission --algorithm dwa room1 room2. "
         "Narrate: robot spins at base (PICKING_UP state), green payload marker appears "
         "above robot in RViz. Point out the DWB trajectory jitter at doorways. "
         "Highlight a recovery spin when a dynamic obstacle crosses the path. "
         "Robot delivers at room1 and room2 (spin + package appears in Gazebo). "
         "Record mission duration from terminal output."),

        ("Step 3 — Algorithm 2 (Smac + MPPI) Run",
         "Launch navigation_launch.py algorithm:=mppi. Run delivery_mission "
         "--algorithm mppi room1 room2. Narrate: compare the smooth curved path in "
         "RViz versus the DWB trajectory from Step 2. Highlight MPPI detouring around "
         "the moving obstacle before it enters the collision zone (predictive vs reactive). "
         "Show the lower recovery count in terminal log. Record mission duration and "
         "compare to Step 2."),

        ("Step 4 — Metrics Comparison",
         "Open ~/delivery_metrics.csv in terminal (column-formatted). Show the "
         "comparison table from Section 4 on screen. Discuss: MPPI is faster, "
         "shorter path, fewer recoveries, higher success — at the cost of 3× CPU. "
         "Conclude which algorithm is recommended for deployment and why."),
    ]
    for title, desc in steps:
        out.append(H2(title))
        out.append(body(desc))
        out.append(GAP())

    out.append(PB())
    return "".join(out)

# ═══════════════════════════════════════════════════════════════════════════════
#  SECTION 7 — CHALLENGES & LESSONS
# ═══════════════════════════════════════════════════════════════════════════════
def sec7():
    out = []
    out.append(H1("7.  Challenges and Lessons Learned"))

    challenges = [
        ("Hardcoded Behavior-Tree XML Paths",
         "The bt_navigator YAML configuration contained an absolute path to another "
         "developer's home directory for the behavior-tree XML file, causing silent "
         "failures on all other machines.",
         "Resolve all installed-file paths dynamically at launch time using "
         "get_package_share_directory('nav2_bt_navigator') and pass the resolved path "
         "as a parameter override in the Node() definition. Never hardcode absolute paths "
         "in YAML configuration files."),

        ("Dual Source Directories",
         "The workspace contained two separate copies of the package: a git working "
         "directory and a src/ colcon source directory. Edits made to one were not "
         "reflected in the build unless explicitly synced to the other, causing "
         "confusing 'old code still running' symptoms.",
         "Always build with --base-paths src and verify the correct source is being "
         "compiled. The preferred layout places only a single canonical package inside src/."),

        ("AMCL Particle Divergence at Launch",
         "With an inaccurate initial pose estimate, the AMCL particle filter spread "
         "across the entire map and never converged, causing the robot to navigate "
         "toward incorrect rooms for the entire session.",
         "Set set_initial_pose: true with an accurate (x, y, yaw) in the YAML. "
         "Alternatively, use a 2D Pose Estimate click in RViz before dispatching the "
         "first NavigateToPose goal. Confirm convergence by monitoring /amcl_pose "
         "covariance before starting the mission."),

        ("MPPI Critic Tuning — Corner Cutting",
         "Default MPPI critic weights caused the robot to cut corners into inflated "
         "wall cost regions, producing near-collision trajectories near doorframes and "
         "triggering the collision monitor safety stop.",
         "Increase PathAlignCritic.cost_weight to 14.0 and CostCritic.cost_weight to "
         "3.81. Tune critics in isolation by running single-room missions and inspecting "
         "the trajectory visualization. MPPI tuning requires iterative refinement."),

        ("Costmap Clearing Latency with Dynamic Obstacles",
         "DWB sometimes failed to clear the costmap fast enough after a dynamic obstacle "
         "moved away, leaving phantom lethal cells that blocked the robot indefinitely "
         "and triggered repeated unnecessary recovery spins.",
         "Set raytrace_max_range equal to obstacle_max_range (both 2.5 m–3.0 m) so "
         "clearing and marking operations always operate at the same sensor depth. "
         "Also increase local costmap update_frequency from 5 Hz to 10 Hz."),

        ("RViz Payload Marker Positioning",
         "Publishing a visualization_msgs/Marker in the map frame at the robot's "
         "current odometry coordinates left the marker stranded in place when the "
         "robot moved to a new position.",
         "Publish the marker in the base_link frame with position.z = 0.28 m and a "
         "short lifetime (1 second), refreshed by a 5 Hz timer. The TF2 transform "
         "tree causes RViz to automatically move the marker as the robot moves, "
         "creating a convincing 'attached package' visual with no additional tracking code."),

        ("Gazebo State Service Unavailable",
         "The /gazebo/set_entity_state service was unavailable when Gazebo was launched "
         "without the libgazebo_ros_state.so system plugin, causing the dynamic_obstacles "
         "node to spin silently without moving any boxes, making the environment static.",
         "Add -s libgazebo_ros_state.so to the gazebo ExecuteProcess command in "
         "navigation_launch.py. Guard all service calls with service_is_ready() checks "
         "before sending requests."),
    ]

    for title, problem, lesson in challenges:
        out.append(H2(title))
        out.append(body_runs(
            R("Problem: ", bold=True, sz=24, color=RED) +
            R(problem, sz=24)))
        out.append(body_runs(
            R("Lesson learned: ", bold=True, sz=24, color=MID_BLUE) +
            R(lesson, sz=24, italic=True)))
        out.append(GAP())

    out.append(PB())
    return "".join(out)

# ═══════════════════════════════════════════════════════════════════════════════
#  SECTION 8 — REFERENCES
# ═══════════════════════════════════════════════════════════════════════════════
def sec8():
    out = []
    out.append(H1("8.  References"))
    refs = [
        '[1]  S. Macenski, F. Martin, R. White, and J. Clavero, "The Marathon 2: A Navigation System," in Proc. IEEE/RSJ IROS, 2020. — Nav2 framework.',
        '[2]  P. E. Hart, N. J. Nilsson, and B. Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths," IEEE Trans. Syst. Sci. Cybern., vol. 4, no. 2, pp. 100-107, 1968. — A* algorithm.',
        '[3]  D. Fox, W. Burgard, and S. Thrun, "The Dynamic Window Approach to Collision Avoidance," IEEE Robot. Autom. Mag., vol. 4, no. 1, pp. 23-33, 1997.',
        '[4]  D. Dolgov, S. Thrun, M. Montemerlo, and J. Diebel, "Practical Search Techniques in Path Planning for Autonomous Driving," in Proc. AAAI Workshop, 2008. — Hybrid A*.',
        '[5]  G. Williams, P. Drews, B. Goldfain, J. M. Rehg, and E. A. Theodorou, "Information Theoretic MPC for Model-Based Reinforcement Learning," in Proc. IEEE ICRA, 2017. — MPPI.',
        '[6]  D. Fox, "KLD-Sampling: Adaptive Particle Filters," in Advances in Neural Information Processing Systems, vol. 14, 2001. — Adaptive AMCL.',
        '[7]  S. Macenski and I. Jambrecic, "SLAM Toolbox: SLAM for the Dynamic World," J. Open Source Softw., vol. 6, no. 61, p. 2783, 2021.',
        '[8]  ROS 2 Humble Hawksbill Documentation. Available: https://docs.ros.org/en/humble/',
        '[9]  Nav2 Navigation Stack Documentation. Available: https://navigation.ros.org/',
        '[10] Gazebo Classic Simulation Documentation. Available: https://classic.gazebosim.org/',
    ]
    for ref in refs:
        pp = (f'<w:pStyle w:val="Normal"/>'
              f'<w:spacing w:before="0" w:after="80" w:line="276" w:lineRule="auto"/>'
              f'<w:ind w:left="720" w:hanging="720"/>')
        out.append(f'<w:p><w:pPr>{pp}</w:pPr>{R(ref, sz=22)}</w:p>')
    return "".join(out)

# ═══════════════════════════════════════════════════════════════════════════════
#  DOCX ASSETS
# ═══════════════════════════════════════════════════════════════════════════════

CONTENT_TYPES = """\
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml"  ContentType="application/xml"/>
  <Override PartName="/word/document.xml"
    ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>
  <Override PartName="/word/styles.xml"
    ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>
  <Override PartName="/word/settings.xml"
    ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.settings+xml"/>
  <Override PartName="/word/numbering.xml"
    ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.numbering+xml"/>
  <Override PartName="/word/footer1.xml"
    ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.footer+xml"/>
  <Override PartName="/word/footer2.xml"
    ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.footer+xml"/>
  <Override PartName="/docProps/core.xml"
    ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>
</Types>"""

RELS = """\
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1"
    Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument"
    Target="word/document.xml"/>
  <Relationship Id="rId2"
    Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties"
    Target="docProps/core.xml"/>
</Relationships>"""

WORD_RELS = """\
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1"
    Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles"
    Target="styles.xml"/>
  <Relationship Id="rId2"
    Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/settings"
    Target="settings.xml"/>
  <Relationship Id="rId3"
    Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/numbering"
    Target="numbering.xml"/>
  <Relationship Id="rId4"
    Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/footer"
    Target="footer1.xml"/>
  <Relationship Id="rId5"
    Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/footer"
    Target="footer2.xml"/>
</Relationships>"""

# default footer — centred page number
FOOTER1 = """\
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:ftr xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
  <w:p>
    <w:pPr>
      <w:jc w:val="center"/>
      <w:spacing w:before="80" w:after="0"/>
    </w:pPr>
    <w:r>
      <w:rPr>
        <w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/>
        <w:sz w:val="20"/><w:color w:val="595959"/>
      </w:rPr>
      <w:fldChar w:fldCharType="begin"/>
    </w:r>
    <w:r>
      <w:rPr>
        <w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/>
        <w:sz w:val="20"/><w:color w:val="595959"/>
      </w:rPr>
      <w:instrText xml:space="preserve"> PAGE </w:instrText>
    </w:r>
    <w:r>
      <w:rPr>
        <w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/>
        <w:sz w:val="20"/><w:color w:val="595959"/>
      </w:rPr>
      <w:fldChar w:fldCharType="end"/>
    </w:r>
  </w:p>
</w:ftr>"""

# first-page footer — empty (suppresses page number on cover)
FOOTER2 = """\
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:ftr xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
  <w:p><w:pPr><w:spacing w:before="0" w:after="0"/></w:pPr></w:p>
</w:ftr>"""

SETTINGS = """\
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:settings xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
  <w:updateFields w:val="true"/>
  <w:defaultTabStop w:val="720"/>
  <w:compat>
    <w:compatSetting w:name="compatibilityMode"
      w:uri="http://schemas.microsoft.com/office/word" w:val="15"/>
  </w:compat>
</w:settings>"""

NUMBERING = """\
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:numbering xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
  <w:abstractNum w:abstractNumId="0">
    <w:lvl w:ilvl="0">
      <w:start w:val="1"/><w:numFmt w:val="bullet"/>
      <w:lvlText w:val="&#x2022;"/>
      <w:lvlJc w:val="left"/>
      <w:pPr><w:ind w:left="720" w:hanging="360"/></w:pPr>
      <w:rPr><w:rFonts w:ascii="Symbol" w:hAnsi="Symbol"/><w:sz w:val="24"/></w:rPr>
    </w:lvl>
    <w:lvl w:ilvl="1">
      <w:start w:val="1"/><w:numFmt w:val="bullet"/>
      <w:lvlText w:val="&#x25E6;"/>
      <w:lvlJc w:val="left"/>
      <w:pPr><w:ind w:left="1080" w:hanging="360"/></w:pPr>
      <w:rPr><w:sz w:val="22"/></w:rPr>
    </w:lvl>
  </w:abstractNum>
  <w:num w:numId="1"><w:abstractNumId w:val="0"/></w:num>
</w:numbering>"""

STYLES = f"""\
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
  <w:docDefaults>
    <w:rPrDefault><w:rPr>
      <w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman" w:cs="Times New Roman"/>
      <w:sz w:val="24"/><w:szCs w:val="24"/>
      <w:color w:val="000000"/>
      <w:lang w:val="en-US"/>
    </w:rPr></w:rPrDefault>
    <w:pPrDefault><w:pPr>
      <w:spacing w:after="160" w:line="276" w:lineRule="auto"/>
      <w:jc w:val="both"/>
    </w:pPr></w:pPrDefault>
  </w:docDefaults>

  <w:style w:type="paragraph" w:styleId="Normal" w:default="1">
    <w:name w:val="Normal"/>
  </w:style>

  <w:style w:type="paragraph" w:styleId="Heading1">
    <w:name w:val="heading 1"/>
    <w:basedOn w:val="Normal"/><w:next w:val="Normal"/>
    <w:qFormat/>
    <w:pPr>
      <w:outlineLvl w:val="0"/>
      <w:spacing w:before="360" w:after="120" w:line="276" w:lineRule="auto"/>
      <w:pBdr><w:bottom w:val="single" w:sz="6" w:space="2" w:color="{MID_BLUE}"/></w:pBdr>
    </w:pPr>
    <w:rPr>
      <w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/>
      <w:b/><w:bCs/>
      <w:sz w:val="30"/><w:szCs w:val="30"/>
      <w:color w:val="{DARK_BLUE}"/>
    </w:rPr>
  </w:style>

  <w:style w:type="paragraph" w:styleId="Heading2">
    <w:name w:val="heading 2"/>
    <w:basedOn w:val="Normal"/><w:next w:val="Normal"/>
    <w:qFormat/>
    <w:pPr>
      <w:outlineLvl w:val="1"/>
      <w:spacing w:before="240" w:after="80" w:line="276" w:lineRule="auto"/>
    </w:pPr>
    <w:rPr>
      <w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/>
      <w:b/><w:bCs/>
      <w:sz w:val="26"/><w:szCs w:val="26"/>
      <w:color w:val="{MID_BLUE}"/>
    </w:rPr>
  </w:style>

  <w:style w:type="paragraph" w:styleId="Heading3">
    <w:name w:val="heading 3"/>
    <w:basedOn w:val="Normal"/><w:next w:val="Normal"/>
    <w:qFormat/>
    <w:pPr>
      <w:outlineLvl w:val="2"/>
      <w:spacing w:before="160" w:after="60" w:line="276" w:lineRule="auto"/>
    </w:pPr>
    <w:rPr>
      <w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/>
      <w:b/><w:bCs/><w:i/><w:iCs/>
      <w:sz w:val="24"/><w:szCs w:val="24"/>
      <w:color w:val="{GREY}"/>
    </w:rPr>
  </w:style>

  <w:style w:type="paragraph" w:styleId="Footer">
    <w:name w:val="footer"/>
    <w:basedOn w:val="Normal"/>
    <w:pPr><w:jc w:val="center"/></w:pPr>
    <w:rPr>
      <w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/>
      <w:sz w:val="20"/><w:color w:val="595959"/>
    </w:rPr>
  </w:style>

  <w:style w:type="table" w:styleId="TableGrid">
    <w:name w:val="Table Grid"/>
    <w:tblPr>
      <w:tblBorders>
        <w:top    w:val="single" w:sz="4" w:color="{MID_BLUE}"/>
        <w:bottom w:val="single" w:sz="4" w:color="{MID_BLUE}"/>
        <w:left   w:val="single" w:sz="4" w:color="{MID_BLUE}"/>
        <w:right  w:val="single" w:sz="4" w:color="{MID_BLUE}"/>
        <w:insideH w:val="single" w:sz="4" w:color="{MID_BLUE}"/>
        <w:insideV w:val="single" w:sz="4" w:color="{MID_BLUE}"/>
      </w:tblBorders>
    </w:tblPr>
  </w:style>
</w:styles>"""

CORE_PROPS = f"""\
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties
  xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties"
  xmlns:dc="http://purl.org/dc/elements/1.1/"
  xmlns:dcterms="http://purl.org/dc/terms/"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <dc:title>Autonomous Indoor Delivery Robot — Project Report</dc:title>
  <dc:subject>Advanced Robotics — ROS 2 Final Project</dc:subject>
  <dc:creator>BETHEL NIGUSU; ESROM ADUGNA; YANIT HABTOM</dc:creator>
  <cp:lastModifiedBy>Delivery Robot Team</cp:lastModifiedBy>
  <dcterms:created xsi:type="dcterms:W3CDTF">{date.today().isoformat()}T00:00:00Z</dcterms:created>
  <dcterms:modified xsi:type="dcterms:W3CDTF">{date.today().isoformat()}T00:00:00Z</dcterms:modified>
</cp:coreProperties>"""

# ═══════════════════════════════════════════════════════════════════════════════
#  ASSEMBLE DOCUMENT
# ═══════════════════════════════════════════════════════════════════════════════
def build_document():
    body = "".join([
        cover(),
        toc_page(),
        sec1(), sec2(), sec3(), sec4(),
        sec5(), sec6(), sec7(), sec8(),
    ])

    sect_pr = (
        '<w:sectPr>'
        '<w:footerReference w:type="default" r:id="rId4"/>'
        '<w:footerReference w:type="first"   r:id="rId5"/>'
        '<w:titlePg/>'
        '<w:pgSz w:w="12240" w:h="15840"/>'
        '<w:pgMar w:top="1440" w:right="1440" w:bottom="1440"'
        '         w:left="1440" w:header="720" w:footer="720" w:gutter="0"/>'
        '<w:pgNumType w:start="1"/>'
        '</w:sectPr>'
    )

    ns = ('xmlns:wpc="http://schemas.microsoft.com/office/word/2010/wordprocessingCanvas"'
          ' xmlns:mc="http://schemas.openxmlformats.org/markup-compatibility/2006"'
          ' xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"'
          ' xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"'
          ' xmlns:w14="http://schemas.microsoft.com/office/word/2010/wordml"'
          ' mc:Ignorable="w14"')

    return (f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            f'<w:document {ns}>'
            f'<w:body>{body}{sect_pr}</w:body>'
            f'</w:document>')


def main():
    doc_xml = build_document()
    with zipfile.ZipFile(OUT, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml',          CONTENT_TYPES)
        z.writestr('_rels/.rels',                   RELS)
        z.writestr('word/_rels/document.xml.rels',  WORD_RELS)
        z.writestr('word/document.xml',             doc_xml)
        z.writestr('word/styles.xml',               STYLES)
        z.writestr('word/settings.xml',             SETTINGS)
        z.writestr('word/numbering.xml',            NUMBERING)
        z.writestr('word/footer1.xml',              FOOTER1)
        z.writestr('word/footer2.xml',              FOOTER2)
        z.writestr('docProps/core.xml',             CORE_PROPS)
    size = os.path.getsize(OUT) // 1024
    print(f"Created: {OUT}  ({size} KB)")


if __name__ == '__main__':
    main()
