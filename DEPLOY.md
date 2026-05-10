# 🚀 GitHub Deployment Guide

Complete step-by-step instructions to publish your delivery robot project to GitHub.

---

## Step 1 — Create the GitHub Repository

1. Go to [github.com/new](https://github.com/new)
2. Fill in:
   - **Repository name:** `delivery_robot`
   - **Description:** `Autonomous Indoor Delivery Robot — Full-Stack ROS 2 & Gazebo Simulation`
   - **Visibility:** Public (required for portfolio / submission)
   - ❌ Do **not** initialise with README, .gitignore, or licence (we have our own)
3. Click **Create repository**

---

## Step 2 — Initialise Git Locally

```bash
cd ~/delivery_ws/src/delivery_robot

# Initialise repository
git init
git checkout -b main

# Configure your identity (if not already done globally)
git config user.name  "Your Name"
git config user.email "you@example.com"
```

---

## Step 3 — Stage and Commit All Files

```bash
# Verify .gitignore is working (build/ install/ log/ should NOT appear)
git status

# Stage everything
git add .

# First commit — conventional commit format
git commit -m "feat: initial autonomous delivery robot ROS 2 package

- Custom differential-drive URDF with 360° LIDAR and IMU
- Multi-room 10x8m Gazebo office world (4 rooms, furniture, dynamic obstacle)
- slam_toolbox online-async mapping pipeline with loop closure
- Full Nav2 stack: AMCL · NavFn A* · DWB · behaviour-tree navigator
- Autonomous delivery mission node with multi-stop routing and fault recovery
- Waypoint recorder and health-check utilities
- RViz2 configs for both SLAM and navigation modes
- Unit tests (pytest) for mission node
- Professional README with architecture diagrams and troubleshooting guide"
```

---

## Step 4 — Connect to GitHub and Push

```bash
# Replace YOUR_USERNAME with your actual GitHub username
git remote add origin https://github.com/YOUR_USERNAME/delivery_robot.git

# Push to GitHub
git push -u origin main
```

If you use SSH keys instead of HTTPS:
```bash
git remote add origin git@github.com:YOUR_USERNAME/delivery_robot.git
git push -u origin main
```

---

## Step 5 — Add a Demo GIF (High Impact)

A GIF showing the robot navigating is the single most impactful addition to your repo.

```bash
# Install screen recorder
sudo apt install kazam   # or: sudo apt install peek

# Record your screen while running the navigation demo
# Then convert the video to GIF:
sudo apt install ffmpeg
ffmpeg -i demo.mp4 -vf "fps=10,scale=800:-1" -loop 0 docs/demo.gif

# Add to git
mkdir -p docs
git add docs/demo.gif
git commit -m "docs: add navigation demo GIF"
git push
```

Then update `README.md` to reference it:
```markdown
![Demo](docs/demo.gif)
```

---

## Step 6 — Add GitHub Topics

On your repository page:
1. Click the ⚙️ gear icon next to "About"
2. Add topics: `ros2`, `robotics`, `gazebo`, `navigation`, `slam`, `nav2`, `python`, `autonomous-robot`, `simulation`

---

## Step 7 — Protect the Main Branch (Team Projects)

In **Settings → Branches → Add rule**:
- Branch name pattern: `main`
- ✅ Require pull request reviews before merging
- ✅ Require status checks to pass
- ✅ Include administrators

---

## Step 8 — Ongoing Development Workflow

```bash
# Create feature branch for each new feature
git checkout -b feat/voice-control

# Work, commit frequently
git add -p                          # stage interactively
git commit -m "feat: add speech_recognition waypoint commands"

# Push branch and open Pull Request on GitHub
git push origin feat/voice-control

# After review and merge, sync main
git checkout main
git pull origin main
```

### Commit Message Convention

```
feat:     new feature
fix:      bug fix
docs:     documentation only
config:   parameter/config changes
test:     add or fix tests
refactor: code restructure, no behaviour change
chore:    build system, dependencies
```

---

## Step 9 — Release Tagging

When the project is demo-ready:

```bash
git tag -a v1.0.0 -m "Release v1.0.0 — Full mapping and delivery demo"
git push origin v1.0.0
```

On GitHub, go to **Releases → Create release** and attach your demo video.

---

## Checklist Before Submitting

- [ ] `colcon build` completes with no errors or warnings
- [ ] `colcon test` passes all unit tests
- [ ] SLAM launch starts Gazebo + robot + RViz2 correctly
- [ ] Map saved to `maps/` (both `.pgm` and `.yaml`)
- [ ] Navigation launch loads saved map and Nav2 stack
- [ ] `health_check` reports all systems GO
- [ ] `delivery_mission room1 room2` completes successfully
- [ ] Multi-stop route completes and returns to base
- [ ] `.gitignore` excludes `build/`, `install/`, `log/`, `__pycache__/`
- [ ] README has accurate quick-start commands that match your package
- [ ] Demo GIF or video is included in `docs/`
- [ ] All team members are added as collaborators on GitHub
- [ ] Repository is set to Public
- [ ] Topics/tags added for discoverability
