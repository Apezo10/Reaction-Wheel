# Results

## Hardware Demo

Video: [reaction-wheel-0.7rad-demo.MOV](results/reaction-wheel-0.7rad-demo.MOV)

The hardware demo shows the reaction wheel correcting the rig from about
`0.7 rad` away from the target angle. This is the observed practical correction
range for the current test setup, not a guarantee that the system can recover
from larger disturbances.

## Observed Limitations

The main hardware limit is the low motor speed. The motor is rated at only
`550 rpm` no-load, so the reaction wheel quickly reaches its usable speed range
and becomes saturated. Once the wheel is saturated, the controller can still
request more PWM, but the wheel cannot keep adding useful angular momentum to
pull the body back toward the target.

The motor also has a large PWM deadband. In testing, the motor did not spin
reliably below about PWM `90`, so the firmware raises any nonzero command below
that value to PWM `90`. This helps overcome stiction, but it also makes small
corrections coarse. Around the demonstrated `0.7 rad` offset from the target
angle, the deadband prevents the motor from accurately applying the tiny torque
changes needed for smooth final correction.
