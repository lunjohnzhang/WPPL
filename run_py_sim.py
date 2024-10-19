import sys
sys.path.append('build')
sys.path.append('scripts')
import fire
import json
import py_sim  # type: ignore # ignore pylance warning


def main():
    simulator = py_sim.py_sim(
    )
    print(simulator.warmup())

if __name__ == '__main__':
    fire.Fire(main)