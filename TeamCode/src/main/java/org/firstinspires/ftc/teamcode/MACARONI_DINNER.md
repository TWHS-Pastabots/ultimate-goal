# Macaroni Dinner Robot

This is basic documentation describing motor/servo pin-outs and a summary of the control behavior for the Macaroni Dinner Robot for TWHS Pastabots.

## Control Hub Connection

To connect to the control hub, connect to `FTC-Rz3m` with password `password`.

To connect ADB wirelessly, execute `adb connect 192.168.43.1:5555`.

To connect to the browser dashboard, enter `http://192:168.43.1:8080` in your browser.

## Pin-outs

Hub Pins are labeled by 1 to 2 letters followed by a number. The letter prefixing the pin-out label
describes which hub (and where) it goes

- The **C** prefix is for the **Control Hub**.
- The **E** prefix is for the **Expansion Hub**.
- The optional **S** prefix is for the pin-outs relating to servos.


| Component Name        | Hub Pin   | Configuration ID |
|-----------------------|-----------|------------------|
| Left Front Motor      | C3        | leftFront        |
| Left Rear Motor       | C0        | leftRear         |
| Right Front Motor     | C2        | rightFront       |
| Right Rear Motor      | C1        | rightRear        |
| Conveyor Belt Motor   | E1        | beltMotor        |
| Intake Motor          | E2        | intakeMotor      |
| Launcher Motor        | E3        | launcherMotor    |
| Intake Servo          | ES0       | intakeServo      |
| Conveyor Belt Stopper | ES2       | stopperServo     |
| Wobble Goal Servo     | ES1       | wobbleServo      |

## Controller Behavior

The left stick controls the Robot's movement.
The right stick controls the Robot's rotation.