builtin_interfaces/Time stamp
string module_name
uint8 level
uint8 OK=0
uint8 WARN=1
uint8 ERROR=2
# HH_260721 - Report READY/STANDBY/ACTIVE/phase text independently from health severity.
string operating_state
string message
string[] missing_nodes
string[] missing_topics
string[] missing_lifecycle_nodes
