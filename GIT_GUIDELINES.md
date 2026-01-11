# 🐙 Git Workflow Guidelines for Bihar UAV Project

This guide outlines the standard Git procedures for managing the autonomous drone codebase.

## 1. Initialize Repository
If you haven't started git yet:

```bash
cd ~/uav_agricultural_drone_project
git init
```

## 2. Configure .gitignore
We must exclude build artifacts (`build/`, `install/`) to keep the repo lightweight.

**Command:**
```bash
touch .gitignore
```

**Content for .gitignore:**
```text
# ROS 2 Build System
build/
install/
log/

# Python
__pycache__/
*.pyc
*.pyo
venv/
.venv/

# Large Binaries
QGroundControl.AppImage

# Drone Logs
*.ulg
*.bag
*.mcap

# IDEs
.vscode/
.idea/
```

## 3. Stage and Commit
Save your progress frequently.

```bash
# 1. Check what changed
git status

# 2. Stage all files (respecting .gitignore)
git add .

# 3. Commit with a clear message
git commit -m "feat: Added YOLOv8 detection node"
```

## 4. Push to GitHub
Upload your code to the remote repository.

**Repository:** `https://github.com/TheAbhishekraj/Biharexport`

```bash
# 1. Rename default branch to main
git branch -M main

# 2. Link to your GitHub repo
git remote add origin https://github.com/TheAbhishekraj/Biharexport.git

# 3. Push for the first time
git push -u origin main
```

## 5. Routine Workflow
For daily updates:
```bash
git add .
git commit -m "fix: Adjusted PID gains for 15kg payload"
git push
```