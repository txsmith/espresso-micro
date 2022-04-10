# espresso-µ

An effort to make my espresso machine a little bit smarter

TODO:
- Emit heater power
- Emit logs/errors
- Control pump pwr via BLE
- make a test-setup for pump action
- Chart heat

## Flow/presure control API
Requirements:
- R1: Controlling pressure over time
  - R1.1: Measure pressure and engage pump to reach goal pressure
- R2: Controlling flow rate over time
  - R2.1: Derive flow rate and engage pump to reach goal flow
- R3: Allow changing between pressure/flow control during the shot
- R4: Allow a dynamic profile: "ramp to 9 bars and hold current flow rate"
- R5: Allow stopping the shot after specified time / output weight

A shot profile is defined as:
A list of steps where each step consists of:
- A directive:
  - Reach pressure of 9 bars
  - Reach flow of 2 ml/s
  - Ramp pressure/flow up/down by X bars/s / X ml/s^2
  - Idle
- An exit condition
  - Shot duration = X
  - Step duration = X
  - Pressure is X bar
  - Flow is X ml/s
  - X or Y

```
ProfileExp := Shot name: [StepExp+]
StepExp := - [DirectiveExp]

DirectiveExp :=
  | Pressure <Current|int bar> (SmoothExp) (until [ExitExp])
  | <Increase|Decrease> Pressure int bar/s until [ExitExp]
  | Flow <Current|int ml/s> (SmoothExp) (until [ExitExp])
  | <Increase|Decrease> Flow int ml/s^2 until [ExitExp]

SmoothExp :=
  | Smooth
  | Medium
  | Harsh

ExitExp :=
  | ShotTime [time]
  | StepTime [time]
  | int bar
  | int ml/s

Shot myShotProfile:
  - Pressure 9 bar
  - Flow Current until ShotTime 30 seconds
```
