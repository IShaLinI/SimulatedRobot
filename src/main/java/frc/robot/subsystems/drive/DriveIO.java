package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs implements LoggableInputs{
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double gyroYawRad = 0.0;


    @Override
    public void toLog(LogTable table) {
      // TODO Auto-generated method stub
      
    }
    @Override
    public void fromLog(LogTable table) {
      // TODO Auto-generated method stub
      
    }
    
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {
  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {
  }
}
