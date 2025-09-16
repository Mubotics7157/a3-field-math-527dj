# Assignment 3: Field Math Fundamentals (Guided)

## Overview
You now have the scaffolding to add triggers and tweak drive behaviour. Before we layer on bigger autonomy features, slow down and become fluent with how WPILib expresses field coordinates, angles, and units. This workbook-style assignment walks you through observing the coordinate frame, logging values in multiple units, and building a field-relative "drive to waypoint" command. Each stage adds one idea at a time and leaves room for reflection.

### How to Use This Workbook
- Treat each numbered task as something to attempt **before** you open the solution block for that stage.
- You can reveal the fully working code at any time, but try to predict what it should look like and write your own version first.
- Keep AdvantageScope and the WPILib docs open while you work—using the docs is part of the learning goal.
- When you need coordinates for a location, drive there in sim and read `RealOutputs/Odometry/Robot` so your expected values match reality.

## Required Reading (skim first!)
- [WPILib Coordinate System](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
- [WPILib Java Units Library](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html#the-java-units-library)

### Field Coordinate System Recap
- WPILib's field origin $(0, 0)$ is at the bottom-left corner when you stand at the blue alliance wall looking downfield.
- The **X axis** points away from you (downfield) and the **Y axis** points to your left; this is a right-handed coordinate system, so the **Z axis** points upward.
- Rotations use the right-hand rule: counter-clockwise is positive. Facing straight downfield is `Rotation2d.fromDegrees(0)`.
- A `Pose2d` is a pair $(x, y)$ plus a rotation: $\text{Pose2d}(x_m, y_m, \theta)$.

### Translation, Rotation, Pose Helpers
- `Translation2d` represents a displacement vector on the field. You will use it anytime you care about the direction or distance between two points.
- `Rotation2d` wraps angles and guarantees the sine/cosine stay normalized. Prefer `Rotation2d.fromDegrees()` or `Rotation2d.fromRadians()` instead of handling degrees yourself.
- `Pose2d.relativeTo(otherPose)` computes the relative pose difference. It effectively returns the transform required to get from `otherPose` to the current pose.

### Units Library (WPILib 2025)
- Measures are strongly-typed: `Distance` is different from `LinearVelocity`. This prevents accidental comparisons between metres and metres-per-second.
- Create values using factory methods, e.g. `Meters.of(3.0)` or `Inches.of(12.0)`.
- Convert with `.in(TargetUnit)`, e.g. `Meters.of(3.0).in(Feet)`.
- When you need the raw number again, store the result of `.in(...)` in a `double`. Do **not** cache the measure itself in a primitive.

### Vector Math Preview
To point the robot from its current pose to a target pose, we use vector subtraction:
```math
\vec{d} = \begin{bmatrix} x \\ y \end{bmatrix}_{\text{target}} - \begin{bmatrix} x \\ y \end{bmatrix}_{\text{current}}
```
The unit (direction-only) vector is:
```math
\hat{d} = \frac{\vec{d}}{\lVert \vec{d} \rVert} = \frac{1}{\sqrt{x^2 + y^2}} \begin{bmatrix} x \\ y \end{bmatrix}
```
When the distance $\lVert \vec{d} \rVert$ is below some tolerance, you can stop translating altogether.

Keep these equations nearby—they explain the `Translation2d` normalization you will code later.

## What You'll Build
By the end you will have:
- Telemetry logging pose data in metres, feet, inches, and heading so you can overlay them in AdvantageScope.
- A `FieldReferencePoses` helper that stores alliance-aware reference poses and reuses the logging helpers.
- A button-triggered command that pushes the robot toward a waypoint until you arrive inside a distance tolerance.

## Pre-Flight Checklist
- [ ] Create a feature branch (for example `feature/assignment3-field-math`).
- [ ] Launch `WPILib: Simulate robot code` and make sure the robot boots in the sim window.
- [ ] Open `RobotContainer.java`, `Drive.java`, `DriveCommands.java`, and eventually the new constants file in your editor.

---

## Stage 1 — Observe the Coordinate Frame
**Goal:** Build trust that the axes in your code match the axes on the real field.

1. Drive the simulated robot in a slow square while watching the field widget. Think of which axis you expect to change when you push each stick direction.
2. In `Drive.periodic()`, log the current pose in metres, feet, inches, and heading. Use the units library helpers described earlier.
3. Restart the simulator and confirm the graphs in AdvantageScope scale exactly (feet = metres × 3.28084, inches = metres × 39.3701). If the graph is flipped, revisit the docs before moving on.
4. Pay attention to which stick input produces a positive heading change—the “spins left vs. right” mental model becomes important later.

<details>
<summary>✅ Stage 1 reference implementation (open after you try)</summary>

```java
Pose2d currentPose = getPose();
Logger.recordOutput("Drive/PoseMeters", currentPose);
Logger.recordOutput(
        "Drive/PoseFeet",
        new Translation2d(
                Meters.of(currentPose.getX()).in(Feet),
                Meters.of(currentPose.getY()).in(Feet)));
Logger.recordOutput(
        "Drive/PoseInches",
        new Translation2d(
                Meters.of(currentPose.getX()).in(Inches),
                Meters.of(currentPose.getY()).in(Inches)));
Logger.recordOutput("Drive/HeadingDegrees", currentPose.getRotation().getDegrees());
```

Place this snippet near the end of `Drive.periodic()` so it runs once per loop alongside your odometry updates.
</details>

**Checkpoint:** Telemetry values match your mental model and draw smooth traces in AdvantageScope.

💾 **Git checkpoint:** `git commit -am "Log drive pose in multiple units"`

---

## Stage 2 — Define Field Reference Poses with Units
**Goal:** Capture interesting on-field locations once, then reuse them with automatic alliance mirroring.

Before coding, drive the robot to two different field spots in sim and note the pose readout from `RealOutputs/Odometry/Robot` (X, Y, and heading in degrees).

1. Create `src/main/java/frc/robot/constants/FieldReferencePoses.java` with:
   - Field dimensions (length/width) in metres.
   - A pose for the blue-side target, and a mirrored pose for red.
   - A helper that returns the correct pose given an `Alliance`.
   - A `Distance` tolerance method for future commands.
   - A logging helper that records the pose in multiple units (reuse the Stage 1 idea so the code stays consistent).
2. In `RobotContainer`, call the logging helper during construction so the constants pop into AdvantageScope immediately.
3. Flip the alliance in the Driver Station simulator. Think through what changes: X should mirror across field length, heading should rotate 180°.

> **Why mirror the heading too?** Imagine you point at the reef on blue: you face 180°. On red you still want to look at the reef, but now it's behind you relative to the field origin, so you add 180°.

<details>
<summary>✅ Stage 2 reference implementation</summary>

```java
package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.Logger;

public final class FieldReferencePoses {
    private FieldReferencePoses() {}

    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.211;

    public static final Pose2d BLUE_REEF_FACE =
            new Pose2d(6.15, FIELD_WIDTH_METERS / 2.0, Rotation2d.fromDegrees(180));
    public static final Pose2d RED_REEF_FACE = mirrorAcrossFieldLength(BLUE_REEF_FACE);

    public static Pose2d reefFaceForAlliance(Alliance alliance) {
        return alliance == Alliance.Red ? RED_REEF_FACE : BLUE_REEF_FACE;
    }

    public static Distance defaultReefTolerance() {
        return Inches.of(8.0);
    }

    public static void logPose(String baseName, Pose2d pose) {
        Logger.recordOutput(baseName + "/PoseMeters", pose);
        Logger.recordOutput(
                baseName + "/PoseFeet",
                new Translation2d(
                        Meters.of(pose.getX()).in(Feet),
                        Meters.of(pose.getY()).in(Feet)));
        Logger.recordOutput(
                baseName + "/PoseInches",
                new Translation2d(
                        Meters.of(pose.getX()).in(Inches),
                        Meters.of(pose.getY()).in(Inches)));
        Logger.recordOutput(baseName + "/HeadingDeg", pose.getRotation().getDegrees());
    }

    private static Pose2d mirrorAcrossFieldLength(Pose2d pose) {
        double mirroredX = FIELD_LENGTH_METERS - pose.getX();
        Rotation2d mirroredHeading = pose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
        return new Pose2d(mirroredX, pose.getY(), mirroredHeading);
    }
}
```

In `RobotContainer` construction:

```java
FieldReferencePoses.logPose("Field/BlueReef", FieldReferencePoses.BLUE_REEF_FACE);
FieldReferencePoses.logPose("Field/RedReef", FieldReferencePoses.RED_REEF_FACE);
```
</details>

**Checkpoint:** Both logged poses appear in AdvantageScope, and mirroring matches the sim values you recorded.

💾 **Git checkpoint:** `git commit -am "Add alliance-aware field reference poses"`

---

## Stage 3 — Calculate a Direction Vector
**Goal:** Convert the pose difference into a direction to travel while respecting a tolerance.

Concept refresh:
- Using the vector formula above, `Pose2d.relativeTo` gives the delta pose in the target's frame.
- `Translation2d.getNorm()` returns $\lVert \vec{d} \rVert$.
- Dividing by the norm gives the unit vector $\hat{d}$.

1. In `DriveCommands`, create a method `fieldVectorTo(currentPose, targetPose, tolerance)`.
2. Use `.relativeTo` and `Translation2d` to compute the delta, log the distance, and return a zero vector when inside tolerance.
3. When outside the tolerance, divide the translation by its norm to get the unit direction vector and log it.
4. Temporarily bind the method to a button (for example `A` button prints the vector) so you can watch the logs update in sim. Expect positive X = forward, positive Y = left when you are on blue alliance.
5. Remove the temporary binding once you trust the data.

> **Debugging tip:** If the vector points the wrong way, print the intermediate `delta` before normalization. A flipped sign usually means the subtraction order is reversed.

<details>
<summary>✅ Stage 3 reference implementation</summary>

```java
public static Translation2d fieldVectorTo(
        Pose2d currentPose, Pose2d targetPose, Distance tolerance) {
    Pose2d delta = targetPose.relativeTo(currentPose);
    Translation2d translation = new Translation2d(delta.getX(), delta.getY());
    double distance = translation.getNorm();
    Logger.recordOutput("Drive/TargetDeltaMeters", distance);
    if (distance <= tolerance.in(Meters)) {
        Translation2d zero = new Translation2d();
        Logger.recordOutput("Drive/TargetDirection", zero);
        return zero;
    }
    Translation2d direction = translation.div(distance);
    Logger.recordOutput("Drive/TargetDirection", direction);
    return direction;
}
```
</details>

**Checkpoint:** With the robot north-west of the target, you see positive X and negative Y until the robot stops inside the tolerance.

💾 **Git checkpoint:** `git commit -am "Add field-relative direction helper"`

---

## Stage 4 — Drive to a Waypoint
**Goal:** Point the robot toward the waypoint and translate until you reach the tolerance, then stop cleanly.

1. In `DriveCommands`, write a command method `driveTowardWaypoint` that:
   - Accepts the `Drive` subsystem, a `Supplier<Pose2d>` for the target, a `Distance` tolerance, and a `LinearVelocity` speed.
   - Calls `fieldVectorTo` each loop to find the direction vector.
   - Multiplies the direction components by the desired speed (converted to metres per second).
   - Uses `Commands.run` to stream velocity commands until the tolerance is satisfied or a timeout occurs.
   - Calls `drive.stop()` in `finallyDo`.
2. Bind the command in `RobotContainer.configureButtonBindings` to a control (for example, the `A` button).
3. In simulation, start a few metres away and press the button. Watch the logs:
   - `Drive/WaypointWithinTolerance` should go true when you arrive.
   - `Drive/WaypointCommandFinished` should record a timestamp every time the command finishes (either success or timeout).
4. Nudge the sticks while the command runs; because the command still holds the `Drive` subsystem, it will keep translating and ignoring driver input until the tolerance is met or the timeout fires.
5. Celebrate by moving the target pose to another field location and observe the same behaviour once you restart the command.

> **Speed maths:** The command multiplies the unit vector $\hat{d}$ by the scalar speed $v$. This yields a velocity vector $\vec{v} = v \cdot \hat{d}$ in metres per second.

<details>
<summary>✅ Stage 4 reference implementation</summary>

```java
public static Command driveTowardWaypoint(
        Drive drive,
        Supplier<Pose2d> targetSupplier,
        Distance tolerance,
        LinearVelocity speed) {
    return Commands.run(
                    () -> {
                        Pose2d target = targetSupplier.get();
                        Translation2d direction = fieldVectorTo(drive.getPose(), target, tolerance);
                        drive.runVelocity(new ChassisSpeeds(
                                direction.getX() * speed.in(MetersPerSecond),
                                direction.getY() * speed.in(MetersPerSecond),
                                0.0));
                        Logger.recordOutput("Drive/WaypointWithinTolerance", direction.getNorm() == 0.0);
                    },
                    drive)
            .until(() -> fieldVectorTo(drive.getPose(), targetSupplier.get(), tolerance).getNorm() == 0.0)
            .withTimeout(3.0)
            .finallyDo(() -> {
                drive.stop();
                Logger.recordOutput("Drive/WaypointCommandFinished", Timer.getFPGATimestamp());
            });
}
```

Remember to import `LinearVelocity`, `Distance`, `Timer`, and `Supplier` as needed.

Binding example inside `configureButtonBindings`:

```java
controller.a().onTrue(DriveCommands.driveTowardWaypoint(
        drive,
        () -> FieldReferencePoses.reefFaceForAlliance(
                DriverStation.getAlliance().orElse(Alliance.Blue)),
        FieldReferencePoses.defaultReefTolerance(),
        MetersPerSecond.of(1.0)));
```
</details>

**Checkpoint:** The robot glides toward the waypoint, stops inside the tolerance (or at the timeout), and only releases the subsystem when the command ends.

💾 **Git checkpoint:** `git commit -am "Add drive toward waypoint command"`

---

## Reflection & Submission
- In your PR description, write a short paragraph about how vector math plus units made the waypoint command easier to reason about.
- Include a screenshot or GIF of AdvantageScope showing pose logs and the waypoint command in action.
- Tag a mentor for review when ready.

## Common Pitfalls and How to Recover
- **Import errors:** Double-check you are using `edu.wpi.first.units.measure.*` for the 2025 units library.
- **Inverted axes:** If forward on your controller makes Y grow, you are reading the axes backwards. Revisit Stage 1.
- **Misused tolerance:** Remember to compare the distance (a scalar) to `tolerance.in(Meters)`. Do not compare vectors directly.
- **Alliance mirroring:** X and heading both flip when you swap sides of the field. If your robot drives the wrong way on red, check the mirror helper.
- **Timeouts:** The 3-second timeout prevents runaway commands. Adjust it if your simulated robot is slower than expected.
- **Command still running after joystick input:** The waypoint command owns the drive subsystem, so it keeps executing until tolerance/timeout. Add an interrupt condition if you want manual override.

## Extension Challenges (Optional, but fun)
1. Allow the D-pad to cycle through a list of stored poses; reuse `driveTowardWaypoint` with the currently selected entry.
2. Scale the velocity based on the remaining distance to add a smooth slowdown near the target.
3. Log the target pose onto the field visualization (`FieldSimulation/GoalPose`) so you can watch the robot and goal simultaneously.
4. Replace the timeout with a dynamically calculated value based on distance and speed.

## Testing Your Implementation
1. Build the project:
   - macOS/Linux: `export JAVA_HOME="/path/to/wpilib/2025/jdk" && ./gradlew build`
   - Windows: `set JAVA_HOME=C:\Users\Public\wpilib\2025\jdk && gradlew build`
2. Run simulation: launch the WPILib sim from VS Code.
3. Test the waypoint command: press the A button and observe the robot glide toward the target.
4. Verify telemetry: AdvantageScope should show matching metric/imperial traces and the unit direction vector.
5. Test alliance switching: toggle the alliance in the Driver Station and confirm mirroring works.

## Additional Learning Resources
- [WPILib Units Documentation](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html)
- [WPILib Coordinate System Guide](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
- [AdvantageScope User Guide](https://github.com/Mechanical-Advantage/AdvantageScope)
- [PathPlanner Documentation](https://pathplanner.dev/home.html) — a great next step after you feel confident with field math.

Make sure each checkpoint matches your mental model before you move on. Mastering these fundamentals now pays huge dividends when you start layering in vision and autonomous routines.
