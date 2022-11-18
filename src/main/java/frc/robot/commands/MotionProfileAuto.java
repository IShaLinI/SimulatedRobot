package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class MotionProfileAuto extends SequentialCommandGroup {
  /**
   * Creates a new MotionProfileAuto , which drives forward for three seconds
   * and then runs the flywheel for ten seconds.
   */
  public MotionProfileAuto(Drive drive) {
    addCommands(
      new InstantCommand(() -> drive.resetPose(new Pose2d(2, 2, new Rotation2d()), new Rotation2d(0))),
      new MotionProfileCommand(
        drive, 
        0, 
        List.of(
          new Pose2d(2, 2, new Rotation2d(0)),
          new Pose2d(5, 5, new Rotation2d(90))
        ), 
        0, 
        false
      )
    );
  }
}
