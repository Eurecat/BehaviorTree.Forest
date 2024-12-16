uint8 IDLE = 0
uint8 RUNNING = 1
uint8 SUCCESS = 2
uint8 FAILURE = 3
uint8 SKIPPED = 4
uint8 PAUSED = 5
uint8 CRASHED = 6

uint8 uid
string name
builtin_interfaces/Time time_start
builtin_interfaces/Time time
uint8 status
string file
string details