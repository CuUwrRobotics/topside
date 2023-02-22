# Since inverse trig/exact coordinates are not useful for our arm at the moment,
# this can be used as a direct drive from the controller.

# Arm joint angle in radians, with 0=upright
int8[1] arm_joints

# Claw open (+) or close (-). Full closing speed = 1.
float32 claw_open_close
