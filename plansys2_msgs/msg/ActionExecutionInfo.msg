string NOT_EXECUTED="1"
string EXECUTING="2"
string FAILED="3"
string SUCCEEDED="4"
string CANCELLED="5"

string status

builtin_interfaces/Time start_stamp
builtin_interfaces/Time status_stamp

string action_full_name

string[] waiting_actions

string action
string[] arguments

# predicted duration of the action from the planner
builtin_interfaces/Duration duration

float32 completion
string message_status
