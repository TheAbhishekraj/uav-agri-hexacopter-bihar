#!/bin/bash
# push_to_git.sh - Automates the git workflow for the Bihar UAV project

REMOTE_URL="https://github.com/TheAbhishekraj/Biharexport.git"

# 1. Initialize if needed
if [ ! -d ".git" ]; then
    echo "🗄️ Initializing Git repository..."
    git init
    git branch -M main
fi

# 2. Set Remote if needed
if ! git remote | grep -q "origin"; then
    echo "🔗 Adding remote origin..."
    git remote add origin "$REMOTE_URL"
fi

# 3. Check for .gitignore
if [ ! -f ".gitignore" ]; then
    echo "📝 Creating .gitignore..."
    printf "build/\ninstall/\nlog/\n__pycache__/\n*.pyc\nvenv/\n.venv/\nQGroundControl.AppImage\n" > .gitignore
fi

# 4. Commit and Push
COMMIT_MSG="${1:-"feat: update agricultural drone project files"}"
echo "🚀 Pushing to GitHub with message: $COMMIT_MSG"

git add .
git commit -m "$COMMIT_MSG"

# Note: This will prompt for your GitHub username and Personal Access Token (PAT) 
# if you haven't configured SSH keys.
if git push -u origin main; then
    echo "✅ Successfully pushed to $REMOTE_URL"
else
    echo "❌ Push failed. Please check your internet connection or Personal Access Token."
    exit 1
fi