# Operating mode of the implement (PTO + 3-point hitch).
# Owner: implement_controller_node, which publishes this as the single
# source of truth on /implement/mode with a TRANSIENT_LOCAL (latched) QoS
# so a late-joining subscriber immediately learns the active mode instead
# of guessing until the next change.
#
# Mode changes are REQUESTED through tractor_msgs/srv/SetImplementMode;
# this topic reports the result. Never infer the mode from a service
# request you sent -- a request can be rejected, and this topic is what
# actually took effect.

std_msgs/Header header

uint8 mode
uint8 MODE_MANUAL      = 0   # operator drives the implement directly; the
                             # controller does not act on the planned path
uint8 MODE_AUTONOMOUS  = 1   # path-driven implement control with no
                             # implement-specific agronomy applied
uint8 MODE_PLANTING    = 2   # path-driven, tuned for a planting pass
uint8 MODE_ROTARY_TILL = 3   # path-driven, tuned for a rotary-tiller
                             # (旋耕机) pass: PTO to full speed first,
                             # then lower to working depth

# Human-readable name of `mode` ("manual", "autonomous", "planting",
# "rotary_till"). Redundant with `mode` on purpose: it makes `ros2 topic
# echo` and recorded bags readable without a constant lookup.
string mode_name

# Who asked for the mode currently in effect. Free-form, set by the
# caller of SetImplementMode ("operator_hmi", "task_manager", "param"
# for the bringup default, ...). Recorded so a field log can answer
# "who put the tractor in this mode" without correlating timestamps.
string source

# When the active mode was entered. Distinct from header.stamp, which is
# the publish time -- the mode topic is republished periodically, so
# header.stamp alone cannot tell you how long the mode has been held.
builtin_interfaces/Time since
