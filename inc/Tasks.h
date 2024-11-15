#pragma once
#include "common.h"

struct Task
{
    int task_id;
    int location;
    int t_assigned = -1;
    int t_completed = -1;
    int agent_assigned = -1;
    // Sortation system only:
    // For sortation system: Whether the task is going to recirculation chute
    bool recirc = false;
    // For sortation system: destinatino of the package
    bool package_dest = -1;
    // For sortation system: assinged chute if the goal is an endpoint
    int assigned_chute = -1;
    // For sortation system: whether this task is a dummy waiting task
    bool dummy_waiting = false;

    Task(int task_id, int location): task_id(task_id), location(location) {};
    Task(int task_id, int location, int t_assigned, int agent_assigned,
         bool recirc = false, int package_dest = -1, int assigned_chute = -1,
         bool dummy_waiting = false):
        task_id(task_id), location(location), t_assigned(t_assigned),
        agent_assigned(agent_assigned), recirc(recirc),
        package_dest(package_dest), assigned_chute(assigned_chute),
        dummy_waiting(dummy_waiting) {};
};
