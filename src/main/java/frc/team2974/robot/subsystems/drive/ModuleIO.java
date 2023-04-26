package frc.team2974.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};
    public double[] driveTempCelcius = new double[] {};

    public double steerAbsolutePositionRad = 0.0;
    public double steerPositionRad = 0.0;
    public double steerVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double[] steerCurrentAmps = new double[] {};
    public double[] steerTempCelcius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the steer motor at the specified voltage. */
  public default void setSteerVoltage(double volts) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setSteerBrakeMode(boolean enable) {}

  // /**
  //  *  Special case features - i.e. off-board closed loop control
  //  */

  // /** Run the drive motor in off-board closed loop velocity control, in meters per second */
  // public default void setOffboardDriveVelocity(double metersPerSec) {}

  // /** Run the steer motor in off-board closed loop position control, in degrees module angle */
  // public default void setOffboardSteerAngle(double degrees) {}

  // /** Set on-motor PID values */
  // public default void setOffboardDrivePID(double kP, double kD) {}

  // /** Set on-motor PID values */
  // public default void setOffboardSteerPID(double kP, double kD) {}
}