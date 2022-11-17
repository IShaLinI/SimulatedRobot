package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;

public class DriveWithFlywheelAuto extends SequentialCommandGroup {
  private static final double drivePercent = 1.0;
  private static final double driveDuration = 5.0;
  private static final double flywheelSpeed = 3000.0;
  private static final double flywheelDuration = 10.0;

  /**
   * Creates a new DriveWithFlywheelAuto, which drives forward for three seconds
   * and then runs the flywheel for ten seconds.
   */
  public DriveWithFlywheelAuto(Drive drive, Flywheel flywheel) {
    addCommands(
        new InstantCommand(() -> drive.resetPose(new Pose2d(2,2, new Rotation2d()), new Rotation2d(30))),
        new StartEndCommand(() -> drive.drivePercent(drivePercent, drivePercent), drive::stop, drive)
            .withTimeout(driveDuration),
        new StartEndCommand(() -> flywheel.runVelocity(flywheelSpeed), flywheel::stop, flywheel)
            .withTimeout(flywheelDuration));
  }
}
