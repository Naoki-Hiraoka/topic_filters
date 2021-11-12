# wrench_filter

## wrench_filter
original code is in https://github.com/ishiguroJSK/hrpsys-base/blob/wbms-dev/rtc/HapticController/HapticController.cpp

### Subscribed Topics:
- `~input` (`geometry_msgs/WrenchStamped`)

### Published Topics:
- `~output` (`geometry_msgs/WrenchStamped`)

### Param
- `~hpf_cutoff_hz` (double)
- `~lpf_cutoff_hz` (double)
- `~gain` (double)
- `~hpf_gain` (double)
- `~lpf_gain` (double)
- `~rate` (double)