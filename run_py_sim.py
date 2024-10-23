import sys

sys.path.append('build')
sys.path.append('scripts')
import fire
import json
import numpy as np
from py_sim import py_sim  # type: ignore # ignore pylance warning


def main():
    with open("demo_kwargs.json") as f:
        kwargs = json.load(f)
    weights = np.ones(len(json.loads(kwargs["weights"]))).tolist()
    wait_costs = np.ones(len(json.loads(kwargs["wait_costs"]))).tolist()
    simulator = py_sim(**kwargs)
    result_warm = simulator.warmup()
    result_warm = json.loads(result_warm)
    result_step = simulator.update_gg_and_step(weights, wait_costs)
    result_step = json.loads(result_step)


if __name__ == '__main__':
    fire.Fire(main)