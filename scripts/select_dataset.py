#!/usr/bin/env python3
"""Activate the three runtime configuration files for one supported dataset."""
import argparse
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', choices=[f'P{i}' for i in range(1, 14)] + ['B1'] + [f'F{i}' for i in range(1, 5)])
    args = parser.parse_args()
    config = Path(__file__).resolve().parents[1] / 'config'
    # Match the retained experiment harness; no independent P9 EKF preset exists.
    settings = 'P1' if args.dataset in ('P2', 'P9') else args.dataset
    pairs = [
        (config / f'slamSettings_{settings}.yaml', config / 'slamSettings.yaml'),
        (config / f'sonar_to_laserscan_params_{args.dataset}.yaml', config / 'sonar_to_laserscan_params.yaml'),
        (config / f'sonar_slam_backend_params_{args.dataset}.yaml', config / 'sonar_slam_backend_params.yaml'),
    ]
    for source, _ in pairs:
        if not source.is_file():
            parser.error(f'Missing preset: {source}')
    for source, target in pairs:
        shutil.copy2(source, target)
        print(f'{source.name} -> {target.name}')
    if settings != args.dataset:
        print(f'Note: {args.dataset} uses the retained {settings} EKF settings; see REPRODUCE.md.')


if __name__ == '__main__':
    main()
