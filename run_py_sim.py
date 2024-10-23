import sys

sys.path.append('build')
sys.path.append('scripts')
import fire
import json
from py_sim import py_sim  # type: ignore # ignore pylance warning


def main():
    with open("demo_kwargs.json") as f:
        kwargs = json.load(f)
    simulator = py_sim(**kwargs)
    result = simulator.warmup()
    breakpoint()
    


if __name__ == '__main__':
    fire.Fire(main)