# Assignment 3: Field Math Fundamentals

## Overview
You now have the scaffolding to add triggers and tweak drive behaviour. Before we layer on bigger autonomy features, slow down and become fluent with how WPILib expresses field coordinates, angles, and units. This assignment guides you through observing the coordinate frame, logging values in multiple units, and building a field-relative "drive to waypoint" command. Each stage is a small turn of the crank—pause at the checkpoints to make sure the data matches your mental model.

### Key Concepts
- Understanding WPILib's field coordinate system:
  - Origin (0,0) is at the bottom-left corner when viewing from behind the blue alliance wall
  - X-axis points downfield (toward the opposite alliance)
  - Y-axis points to the left when standing at the origin looking downfield
  - Positive rotation follows the right-hand rule (counter-clockwise when viewed from above)
- Using the new WPILib Units library for type-safe unit conversions
- Creating field-relative movement commands using vector math
- Building reusable field constants with alliance-aware mirroring

## Required Reading
- WPILib docs — Coordinate System: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
- WPILib docs — Java Units Library: https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html#the-java-units-library

### Important Notes About Units (2025 WPILib)
- The 2025 WPILib units library uses measure types like `Distance` and `LinearVelocity`
- Units are created using static methods like `Meters.of(value)` and `Inches.of(value)`
- Conversions use the `.in()` method: `Meters.of(value).in(Feet)`
- Import paths follow the pattern `edu.wpi.first.units.measure.*`

## Learning Objectives
- Describe how `Pose2d`, `Translation2d`, and `Rotation2d` represent position and heading on the FRC field.
- Convert values between metres, feet, and inches using the Java Units library.
- Derive field-relative direction vectors with `Pose2d.relativeTo` and normalized `Translation2d` math.
- Implement and test a "drive to waypoint" command that stops within a distance tolerance using kinematic math and timeouts.

## What You'll Build
By the end you will have:
- Telemetry logging pose data in both metric and imperial units for AdvantageScope overlays.
- A dedicated constants/utility class that captures alliance-aware reference poses and exposes them in multiple unit systems.
- A button-triggered command that points the robot toward a chosen waypoint and translates until it reaches a configurable distance threshold.

## Pre-Flight Checklist
- [ ] Create a new branch, e.g. `feature/assignment3-field-math`.
- [ ] Launch `WPILib: Simulate robot code` and ensure the robot boots in the sim window.
- [ ] Open `RobotContainer.java`, `DriveCommands.java`, `Drive.java`, and the new constants file side-by-side for quick navigation.

Keep the required reading handy while you work—the docs answer most "why does the axis point that way?" questions.

## Stage 1 — Observe the Coordinate Frame
1. In simulation, drive the robot in a gentle square on the blue end of the field. Pay attention to how the robot icon moves when you push the sticks.
2. In AdvantageScope, watch `FieldSimulation/RobotPosition` and identify which axis grows as you move downfield versus side-to-side.
3. In `Drive.periodic()` (the subsystem already responsible for odometry updates), add something along the lines of:
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
   Logging from the subsystem keeps telemetry tied to the code that owns the data.
4. **Checkpoint 1:** Restart the sim. Confirm the feet and inches traces scale exactly from the metre values and that the heading graph responds to the right stick. If anything looks flipped, revisit the coordinate system article before moving on.

_Commit suggestion:_ once the telemetry graphs look right, make a commit such as `git commit -am "Add drive pose telemetry"` so you can roll back easily later.

## Stage 2 — Define Field Reference Poses with Units
1. Create a new file `src/main/java/frc/robot/constants/FieldReferencePoses.java` (or a similar package that fits your style). Give it a structure like:
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
   Adjust the numbers to wherever you want your waypoint; the mirror helper above assumes the field is symmetric about the X midpoint.
2. In `RobotContainer`, call `FieldReferencePoses.logPose("Field/BlueReef", FieldReferencePoses.BLUE_REEF_FACE);` and the red equivalent during construction so the constants appear in AdvantageScope. Logging once on boot keeps command code uncluttered.
3. Flip the alliance in the Driver Station simulator and confirm the logged pose mirrors. Think through how X and heading change between alliances and tweak the constants if needed.
4. **Checkpoint 2:** With constants and logging in place, the reference poses should match what you see in AdvantageScope. When satisfied, stage and commit the work (e.g. `git commit -am "Add field reference poses"`).

## Stage 3 — Calculate a Direction Vector
1. In `DriveCommands`, add:
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
   Accepting a `Distance` parameter lets callers provide tolerances in any unit.
2. Bind a temporary helper (for instance, `controller.rightBumper().onTrue(Commands.runOnce(() -> fieldVectorTo(...))))`) so you can sample the vector in sim. Watch the `Drive/TargetDirection` log: if the target is forward and left, expect positive X and negative Y. Adjust the math until the signs feel correct, then delete the temporary binding.
3. **Checkpoint 3:** When the logged vector aligns with your intuition from Stage 1, remove the temporary helper and keep this method for the next stage.

## Stage 4 — Drive to a Waypoint
1. Still in `DriveCommands`, implement:
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
   Bring in any missing imports:
   ```java
   import edu.wpi.first.units.measure.Distance;
   import edu.wpi.first.units.measure.LinearVelocity;
   import edu.wpi.first.wpilibj.Timer;
   ```
2. In `RobotContainer.configureButtonBindings`, bind the command to a control such as `controller.a().onTrue(...)`. Pass a supplier that chooses the correct pose for the current alliance, for example:
   ```java
   controller.a().onTrue(DriveCommands.driveTowardWaypoint(
           drive,
           () -> FieldReferencePoses.reefFaceForAlliance(
                   DriverStation.getAlliance().orElse(Alliance.Blue)),
           FieldReferencePoses.defaultReefTolerance(),
           MetersPerSecond.of(1.0)));
   ```
3. **Checkpoint 4:** In simulation, start a few metres from the selected waypoint. Press the button and confirm the robot translates toward the target while holding its current heading. Interrupt the command by moving the sticks; the default drive should reclaim control immediately and the waypoint command should stop logging.

_When everything works end-to-end, make another commit (e.g. `git commit -am "Add driveTowardWaypoint command"`) before opening a PR._

## Reflection & Submission
- In your PR description, explain how the coordinate frame and units helpers made it easier to reason about the "drive to waypoint" logic.
- Include a screenshot or GIF of AdvantageScope showing the pose logs and the approach command in action.
- Tag a mentor for review when ready.

## Common Pitfalls and Solutions
1. **Import Errors**: Make sure to use `edu.wpi.first.units.measure.*` for the 2025 units library
2. **Unit Conversion Syntax**: Use `Meters.of(value).in(TargetUnit)` not `Meters.convertTo()`
3. **Type Safety**: The units library provides compile-time type safety - `Distance` and `LinearVelocity` are distinct types
4. **Alliance Mirroring**: Remember that red alliance needs both X-coordinate and heading mirrored (180° rotation)
5. **Tolerance Check**: The `.getNorm()` method returns the magnitude of a Translation2d vector

## Extension Challenges (Optional)
- Allow the driver to press the D-pad to cycle through a list of stored poses; reuse the same `driveTowardWaypoint` command with the current selection.
- Add a ramped speed profile by scaling the command’s velocity as you approach the target.
- Log the target pose on the field visualization (`FieldSimulation/GoalPose`) so you can watch the robot and goal simultaneously.

## Testing Your Implementation
1. **Build the project**:
   - macOS/Linux: `export JAVA_HOME="/path/to/wpilib/2025/jdk" && ./gradlew build`
   - Windows: `set JAVA_HOME=C:\Users\Public\wpilib\2025\jdk && gradlew build`
2. **Run simulation**: Launch the WPILib simulator from VS Code
3. **Test waypoint command**: Press the A button to drive toward the reef face
4. **Verify telemetry**: Check AdvantageScope for unit conversions and target vectors
5. **Test alliance switching**: Change alliance in Driver Station and verify pose mirroring

Make sure each checkpoint matches your mental model before you move to the next stage. Getting the units and directions right here will pay off when you integrate vision and autonomous paths later.

## Additional Learning Resources
- [WPILib Units Documentation](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html)
- [AdvantageScope User Guide](https://github.com/Mechanical-Advantage/AdvantageScope)
- [PathPlanner Integration](https://pathplanner.dev/home.html) - Next step after mastering field coordinates
