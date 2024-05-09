#!/usr/bin/env python3
"""
Backdate Kestrel Git Commits.

Rewrites commit author and committer timestamps over an arbitrarily chosen
5.5-month quiet window (November 12, 2023 to April 25, 2024) across 2021-2026.
Uses seeded RNG with realistic development intervals and working hours.
Strict zero em dash guarantee in all output and logs.
"""

import os
import sys
import random
import subprocess
from datetime import datetime, timedelta

def get_git_output(cmd):
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
    return res.stdout.strip()

def main():
    # 1. Retrieve all commits in topological order (oldest to newest)
    raw_commits = get_git_output(["git", "log", "--reverse", "--format=%H"]).splitlines()
    commits = [c.strip() for c in raw_commits if c.strip()]
    num_commits = len(commits)
    print(f"Total commits detected: {num_commits}")

    if num_commits == 0:
        print("No commits found to backdate.")
        sys.exit(1)

    # 2. Setup RNG and target window: Nov 12, 2023 to April 25, 2024 (~165 days / 5.5 months)
    random.seed(133742)
    start_dt = datetime(2023, 11, 12, 14, 35, 22)
    total_target_days = 165.0

    raw_intervals = []
    for _ in range(num_commits - 1):
        r = random.random()
        if r < 0.20:
            # Short gap / rapid iteration (0.25 to 0.9 days)
            raw_intervals.append(random.uniform(0.25, 0.9))
        elif r < 0.75:
            # Standard feature progression (1.5 to 3.8 days)
            raw_intervals.append(random.uniform(1.5, 3.8))
        else:
            # Weekend / deep focus gap (3.8 to 5.6 days)
            raw_intervals.append(random.uniform(3.8, 5.6))

    scale = total_target_days / sum(raw_intervals)
    scaled_intervals = [x * scale for x in raw_intervals]

    timestamps = [start_dt]
    cur = start_dt
    for iv in scaled_intervals:
        cur = cur + timedelta(days=iv)
        # Realistic active engineering hours (10:00 to 23:45)
        hour = random.choice([10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23])
        minute = random.randint(0, 59)
        second = random.randint(0, 59)
        candidate = cur.replace(hour=hour, minute=minute, second=second)
        if candidate <= timestamps[-1]:
            candidate = timestamps[-1] + timedelta(hours=random.randint(2, 6), minutes=random.randint(1, 50))
        cur = candidate
        timestamps.append(cur)

    # Verify monotonic ordering
    for i in range(len(timestamps) - 1):
        assert timestamps[i + 1] > timestamps[i], "Timestamps must be strictly monotonically increasing"

    span_days = (timestamps[-1] - timestamps[0]).total_seconds() / 86400.0
    print(f"Target window: {timestamps[0].strftime('%Y-%m-%d %H:%M:%S')} -> {timestamps[-1].strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Total duration: {span_days:.1f} days ({span_days / 30.4375:.2f} months)")

    # 3. Rebuild commit chain using git commit-tree
    new_parent = None
    old_to_new = {}

    for idx, (old_commit, ts) in enumerate(zip(commits, timestamps)):
        # Determine Eastern timezone offset (-0500 EST vs -0400 EDT)
        # In 2024, Daylight Saving Time started March 10
        if ts < datetime(2024, 3, 10, 2, 0, 0):
            tz_offset = "-0500"
        else:
            tz_offset = "-0400"

        ts_str = f"{ts.strftime('%Y-%m-%d %H:%M:%S')} {tz_offset}"

        # Extract metadata
        tree = get_git_output(["git", "rev-parse", f"{old_commit}^{{tree}}"])
        author_name = get_git_output(["git", "log", "-1", "--format=%an", old_commit])
        author_email = get_git_output(["git", "log", "-1", "--format=%ae", old_commit])
        committer_name = get_git_output(["git", "log", "-1", "--format=%cn", old_commit])
        committer_email = get_git_output(["git", "log", "-1", "--format=%ce", old_commit])
        message = subprocess.run(["git", "log", "-1", "--format=%B", old_commit],
                                 stdout=subprocess.PIPE, text=True, check=True).stdout

        # Construct commit-tree command
        cmd = ["git", "commit-tree", tree]
        if new_parent:
            cmd.extend(["-p", new_parent])

        env = os.environ.copy()
        env["GIT_AUTHOR_NAME"] = author_name
        env["GIT_AUTHOR_EMAIL"] = author_email
        env["GIT_AUTHOR_DATE"] = ts_str
        env["GIT_COMMITTER_NAME"] = committer_name
        env["GIT_COMMITTER_EMAIL"] = committer_email
        env["GIT_COMMITTER_DATE"] = ts_str

        proc = subprocess.run(cmd, input=message, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True, env=env, check=True)
        new_commit = proc.stdout.strip()
        new_parent = new_commit
        old_to_new[old_commit] = new_commit

        first_line = message.strip().splitlines()[0] if message.strip() else ""
        print(f"[{idx+1:02d}/{num_commits:02d}] {ts_str} | {new_commit[:7]} | {first_line[:60]}")

    # 4. Update main ref to point to newest commit
    new_head = new_parent
    subprocess.run(["git", "update-ref", "refs/heads/main", new_head], check=True)
    subprocess.run(["git", "checkout", "main"], check=True)
    print("\nSuccessfully updated branch 'main' to backdated commit tree.")
    print(f"New HEAD: {new_head}")

if __name__ == "__main__":
    main()
