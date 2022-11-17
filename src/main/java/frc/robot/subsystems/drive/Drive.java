package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0);

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());

  private SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0, 0, 0);

  /** Creates a new Drive. */
  public Drive(DriveIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0, 0.03);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);

    // Update odometry and log the new pose
    odometry.update(new Rotation2d(-inputs.gyroYawRad), getLeftPositionMeters(), getRightPositionMeters());
    Logger.getInstance().recordOutput("Odometry", getPose());
  }

  /** Run open loop at the specified percentage. */
  public void drivePercent(double leftPercent, double rightPercent) {
    io.setVoltage(leftPercent * 12, rightPercent * 12);
  }

  /** Run open loop based on stick positions. */
  public void driveArcade(double xSpeed, double zRotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    io.setVoltage(speeds.left * 12, speeds.right * 12);
  }

  public void driveVelocity(double leftVelocityMetersPerSec, double rightVelocityMetersPerSec) {
    // Calculate setpoint and feed forward voltage
    double leftVelocityRadPerSec = leftVelocityMetersPerSec / WHEEL_RADIUS_METERS;
    double rightVelocityRadPerSec = rightVelocityMetersPerSec / WHEEL_RADIUS_METERS;
    double leftFFVolts = ffModel.calculate(leftVelocityRadPerSec);
    double rightFFVolts = ffModel.calculate(rightVelocityRadPerSec);

    Logger.getInstance().recordOutput("Drive/LeftSetpointRadPerSec", leftVelocityRadPerSec);
    Logger.getInstance().recordOutput("Drive/RightSetpointRadPerSec", rightVelocityRadPerSec);

    // Use open loop control
    io.setVoltage(leftFFVolts, rightFFVolts);
  }

  /** Stops the drive. */
  public void stop() {
    io.setVoltage(0.0, 0.0);
  }

  /* Sets the robot pose to (0,0,0) */
  public void resetPose() {
    odometry.resetPosition(new Pose2d(), new Rotation2d());
  }

  /* Moves the robot pose but doesnt touch angle */
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(pose, new Rotation2d(-inputs.gyroYawRad));
  }

  /* Sets the robot pose and angle */
  public void resetPose(Pose2d pose, Rotation2d rot) {
    odometry.resetPosition(pose, rot);
  }

  /** Returns the current odometry pose in meters. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Returns the position of the left wheels in meters. */
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * WHEEL_RADIUS_METERS;
  }

  /** Returns the position of the right wheels in meters. */
  public double getRightPositionMeters() {
    return inputs.rightPositionRad * WHEEL_RADIUS_METERS;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  public double getLeftVelocityMeters() {
    return inputs.leftVelocityRadPerSec * WHEEL_RADIUS_METERS;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  public double getRightVelocityMeters() {
    return inputs.rightVelocityRadPerSec * WHEEL_RADIUS_METERS;
  }

  public double getTrackWidthMeters() {
    return 0.61568;
  }

  public double getKs() {
    return ffModel.ks;
  }

  public double getKv() {
    return ffModel.kv;
  }

  public double getKa() {
    return ffModel.ka;
  }

}
