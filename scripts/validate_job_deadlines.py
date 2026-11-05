#!/usr/bin/env python3
"""Verify successful matrix jobs finished within queue-inclusive deadlines."""
from datetime import datetime
import json
import os
import subprocess


def require_timing(job):
    if job.get('status') != 'completed' or job.get('conclusion') != 'success':
        raise ValueError(f"Required job did not succeed: {job.get('name')}")
    try:
        created = datetime.fromisoformat(job['created_at'].replace('Z', '+00:00'))
        completed = datetime.fromisoformat(job['completed_at'].replace('Z', '+00:00'))
        if created.utcoffset() is None or completed.utcoffset() is None:
            raise ValueError('timestamps lack time zones')
        elapsed = (completed - created).total_seconds()
    except (KeyError, AttributeError, TypeError, ValueError) as error:
        raise ValueError(f"Required job has invalid queue or completion time: {job.get('name')}") from error
    if not 0 <= elapsed <= 180:
        raise ValueError(f"Required job exceeded its queue-inclusive 180-second deadline: {job.get('name')} ({elapsed:g}s)")


def validate_paired_consumers(jobs):
    expected = {
        f'{group} ({platform}, {configuration}, v{version}, {version})'
        for group in ('deployed-warm-dependencies', 'deployed-consumer')
        for platform in ('ubuntu-24.04', 'windows-2022')
        for configuration in ('Debug', 'Release')
        for version in ('2026.2.12', '2026.2.15')
    }
    selected = [job for job in jobs if job.get('name') in expected]
    if len(selected) != len(expected) or {job['name'] for job in selected} != expected:
        raise ValueError('Missing or duplicated deployed consumer matrix jobs')
    for job in selected:
        require_timing(job)


def main():
    repository = os.environ['GITHUB_REPOSITORY']
    run_id = os.environ['GITHUB_RUN_ID']
    path = f'repos/{repository}/actions/runs/{run_id}/jobs?filter=latest&per_page=100'
    jobs = json.loads(subprocess.check_output(['gh', 'api', path], text=True))['jobs']
    validate_paired_consumers(jobs)
    print('Validated queue-inclusive deadlines for 16 paired consumer matrix jobs')


if __name__ == '__main__':
    main()
