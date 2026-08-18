# RobotStatus.msg

# Is emergency stop currently active?
bool emergency_stop

# Is the robot currently paused?
bool paused

# Is the robot navigating (moving towards a goal)?
bool navigating

# Current index of the subtask being executed (-1 if none)
int32 current_subtask

# Name of the currently running task file (empty if none)
string current_task_file

# Overall robot state, e.g., "idle", "navigating", "paused", "emergency_stop", "completed"
string state
