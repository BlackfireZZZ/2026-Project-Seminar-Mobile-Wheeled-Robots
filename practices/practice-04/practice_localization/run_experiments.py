import argparse
import csv
import json
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', default='results')
    parser.add_argument('--duration', type=float, default=180.0)
    options = parser.parse_args()
    folder = Path(options.output_dir).resolve()
    folder.mkdir(parents=True, exist_ok=True)
    names = ['initial_small_low', 'initial_small_high', 'initial_large_low',
             'initial_large_high', 'global_low', 'global_high']
    results = []
    for name in names:
        target = folder / f'{name}.json'
        target.unlink(missing_ok=True)
        with (folder / f'{name}.log').open('w') as log:
            subprocess.run([
                'ros2', 'launch', 'practice_localization', 'spawn_robot.launch.py',
                f'experiment:={name}', f'duration:={options.duration}',
                f'output_dir:={folder}', 'mode:=realtime', 'gui:=false', 'use_rviz:=false',
            ], stdout=log, stderr=subprocess.STDOUT, check=True)
        row = json.loads(target.read_text())
        results.append(row)
        print(name, row['convergence_time'], flush=True)
    compare(results, folder)


def compare(results, folder):
    names = [row['experiment_name'] for row in results]
    duration = max(row['duration'] for row in results)
    with (folder / 'comparison.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['experiment', 'min_particles', 'max_particles', 'convergence_time_s'])
        for row in results:
            writer.writerow([row['experiment_name'], row['min_particles'], row['max_particles'], row['convergence_time']])
    fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    for i, row in enumerate(results):
        value = row['convergence_time']
        height = duration if value is None else value
        ax.bar(i, height, color='gray' if value is None else 'steelblue')
        ax.text(i, height, 'Не сошлось' if value is None else f'{value:.1f}',
                ha='center', va='bottom', fontsize=9)
    ax.set_xticks(range(len(names)), names, rotation=25, ha='right')
    ax.set_ylabel('Время сходимости, с')
    ax.set_ylim(0, duration * 1.15)
    ax.grid(axis='y')
    fig.savefig(folder / 'comparison.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
