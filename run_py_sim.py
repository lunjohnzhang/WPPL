import sys

sys.path.append('build')
sys.path.append('scripts')
import fire
import json
import time
import numpy as np
from py_sim import py_sim  # type: ignore # ignore pylance warning


def main():
    with open("demo_kwargs.json") as f:
        kwargs = json.load(f)
    n_valid_edges = len(json.loads(kwargs["weights"]))
    n_wait_costs = len(json.loads(kwargs["wait_costs"]))
    weights = np.ones(n_valid_edges).tolist()
    wait_costs = np.ones(n_wait_costs).tolist()
    simulator = py_sim(**kwargs)
    result_warm = simulator.warmup()
    result_warm = json.loads(result_warm)
    # result_step = simulator.update_gg_and_step(weights, wait_costs)
    # result_step = json.loads(result_step)

    while True:
        # cnt += 1
        start = time.time()

        # dist = simulator.get_tasks_distribution()
        # print(dist[:22])
        # weights = np.random.rand(n_valid_edges).tolist()
        # wait_costs = np.random.rand(n_wait_costs).tolist()
        weights = np.random.rand(n_valid_edges).tolist()
        wait_costs = np.random.rand(n_wait_costs).tolist()
        update_r_str = simulator.update_gg_and_step(weights, wait_costs)
        end = time.time()
        # raise NotImplementedError
        print("Time for last time segment:", end - start)
        update_r = json.loads(update_r_str)

        agents_finish_task = update_r["agents_finish_task"]
        # print(agents_finish_task)
        # final_pos = update_r["final_pos"]
        # print(final_pos)
        # final_task = update_r["final_tasks"]
        # curr_pos = simulator.get_curr_pos()
        # print("final task:", final_task)
        # print("curr pos:", curr_pos)
        # if cnt >= 100:
        #     break
        print("Num task finished:", update_r["num_task_finished"])
        # nt.append(update_r["num_task_finished"])
        if update_r["done"]:
            print("done!")
            break


if __name__ == '__main__':
    fire.Fire(main)
