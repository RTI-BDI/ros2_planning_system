int16 NOT_EXECUTED=1
int16 EXECUTING=2
int16 FAILED=3
int16 SUCCEEDED=4
int16 CANCELLED=5

int16 status

builtin_interfaces/Time start_stamp
builtin_interfaces/Time status_stamp

bool at_start_applied
bool at_end_applied

string action_full_name

string[] waiting_actions

string action
string[] arguments

# predicted duration of the action from the planner
builtin_interfaces/Duration duration

float32 completion
string message_status
