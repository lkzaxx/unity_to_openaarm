#!/bin/bash
set -e
LOGFILE=/home/idaka/.openclaw/logs/auto-update.log
SRCDIR=/home/idaka/openclaw_src
mkdir -p /home/idaka/.openclaw/logs
log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> "$LOGFILE"; }
cd "$SRCDIR"
git fetch origin main 2>/dev/null
LOCAL=$(git rev-parse HEAD)
REMOTE=$(git rev-parse origin/main)
if [ "$LOCAL" = "$REMOTE" ]; then
    log "No updates available"
    exit 0
fi
log "Update: $LOCAL -> $REMOTE"
git stash 2>/dev/null || true
git pull origin main >> "$LOGFILE" 2>&1
log "Building..."
docker build -t openclaw:local . >> "$LOGFILE" 2>&1
log "Restarting..."
docker compose up -d >> "$LOGFILE" 2>&1
log "Done!"
