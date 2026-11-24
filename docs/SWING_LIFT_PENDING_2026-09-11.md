# Withdrawn offline swing-lift candidate

Finalization: user observed a quiet, improved rise on the deployed v57.4.
The unflashed sin^4 candidate and its dedicated test were removed from the
working source to preserve the tested version. The description below is
historical only, not a pending deployment. No firmware upload was performed.

Physical test: CAPTURE -> RISING -> STAND, no reported failsafe; held one
second then /legOff. All 18 PWM enable flags verified false. User reported
no previous trembling/noise in this rise. This does not validate loaded
walking or navigation. Keep outputs off while user mounts the brain.

User confirmed BLS3235 180-degree servos supplied at 6 V. Platform supports
the robot with legs suspended. Prior severe heating remains an unresolved
physical observation; cooling confirmation is needed before the next test.

Source-only change: vertical swing sin(pi*t) -> sin(pi*t)^4. Peak height,
swing duration, IK, offsets, PWM frequency and pulse mapping are unchanged.
The new lift joins the flat stance with zero vertical speed and acceleration.
It does NOT smooth horizontal stance/swing transitions. It concentrates
vertical motion in the middle of swing and may increase peak speed there;
therefore this is a candidate, not a verified cure for trembling or heating.

NOT uploaded to ESP. Live firmware remains the previously flashed v57.4.
Source version still v57.4; assign a new revision before building/deploying.
Do not claim the source modification is running on the robot.
No motor commands were issued during this change.
