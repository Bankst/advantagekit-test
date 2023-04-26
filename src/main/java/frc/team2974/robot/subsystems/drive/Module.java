package frc.team2974.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team2974.robot.RobotConstants;
import frc.team2974.robot.RobotConstants.SwerveModuleK.ModuleConstants;
import frc.team6328.util.LoggedTunableNumber;

import static frc.team2974.robot.RobotConstants.SwerveK;
import static frc.team2974.robot.RobotConstants.SwerveModuleK;

import org.littletonrobotics.junction.Logger;

public class Module {
	private final ModuleIO io_;
	private final ModuleIOInputsAutoLogged inputs_ = new ModuleIOInputsAutoLogged();
	private final int index_;
	private final ModuleConstants constants_;

	private static final LoggedTunableNumber wheelRadius_ =
		new LoggedTunableNumber("Drive/Module/WheelRadius", SwerveK.kWheelDiameterInches / 2);
  private static final LoggedTunableNumber driveKp_ =
		new LoggedTunableNumber("Drive/Module/DriveKp", SwerveModuleK.kDrivePIDConstants.kP());
  private static final LoggedTunableNumber driveKd_ =
		new LoggedTunableNumber("Drive/Module/DriveKd", SwerveModuleK.kDrivePIDConstants.kD());
  private static final LoggedTunableNumber driveKs_ =
		new LoggedTunableNumber("Drive/Module/DriveKs", SwerveModuleK.kDrivePIDConstants.kS());
  private static final LoggedTunableNumber driveKv_ =
		new LoggedTunableNumber("Drive/Module/DriveKv", SwerveModuleK.kDrivePIDConstants.kV());
  private static final LoggedTunableNumber turnKp_ =
		new LoggedTunableNumber("Drive/Module/TurnKp", SwerveModuleK.kSteerPIDConstants.kP());
  private static final LoggedTunableNumber turnKd_ =
		new LoggedTunableNumber("Drive/Module/TurnKd", SwerveModuleK.kSteerPIDConstants.kD());

	private SimpleMotorFeedforward driveFeedforward_ = new SimpleMotorFeedforward(0.0, 0.0);
  private final PIDController driveFeedback_ =
		new PIDController(0.0, 0.0, 0.0, RobotConstants.kLoopPeriodSecs);
  private final PIDController steerFeedback_ =
		new PIDController(0.0, 0.0, 0.0, RobotConstants.kLoopPeriodSecs);

	public Module(ModuleIO io, int index) {
		io_ = io;
		index_ = index;
		constants_ = SwerveModuleK.kModules[index];
		System.out.println("[Module][Ctor] Creating module " +
			Integer.toString(index) + "(" + constants_.name() + ")");

		steerFeedback_.enableContinuousInput(-Math.PI, Math.PI);
	}

	public void periodic() {
		io_.updateInputs(inputs_);
		Logger.getInstance().processInputs("Drive/Module" + Integer.toString(index_), inputs_);

		// Update controllers if tunable numbers have changed
		if (driveKp_.hasChanged(driveKp_.hashCode()) || driveKd_.hasChanged(driveKd_.hashCode())) {
			driveFeedback_.setPID(driveKp_.get(), 0.0, driveKd_.get());
		}

		if (turnKp_.hasChanged(turnKp_.hashCode()) || turnKd_.hasChanged(turnKd_.hashCode())) {
			steerFeedback_.setPID(turnKp_.get(), 0.0, turnKd_.get());
		}

		if (driveKs_.hasChanged(driveKs_.hashCode()) || driveKv_.hasChanged(driveKv_.hashCode())) {
			driveFeedforward_ = new SimpleMotorFeedforward(driveKs_.get(), driveKv_.get());
		}
	}

	public SwerveModuleState runSetpoint(SwerveModuleState state) {
		var optimizedState = SwerveModuleState.optimize(state, getAngle());

		var steerCurAngle = getAngle().getRadians();
		var steerDesAngle = optimizedState.angle.getRadians();
		var steerEffort = steerFeedback_.calculate(steerCurAngle, steerDesAngle);
		Logger.getInstance().recordOutput("Module/"+index_+"/SteerEffort", steerEffort);
		io_.setSteerVoltage(steerEffort);

		// Update velocity based on turn error
    // optimizedState.speedMetersPerSecond *= Math.cos(steerFeedback_.getPositionError());

		// Run drive controller
    double velocityRadPerSec = optimizedState.speedMetersPerSecond / wheelRadius_.get();
		var ffEffort = driveFeedforward_.calculate(velocityRadPerSec);
		var fbEffort = driveFeedback_.calculate(inputs_.driveVelocityRadPerSec, velocityRadPerSec);
		Logger.getInstance().recordOutput("Module/"+index_+"/DriveFFEffort", ffEffort);
		Logger.getInstance().recordOutput("Module/"+index_+"/DriveFBEffort", fbEffort);
		io_.setDriveVoltage(ffEffort + 3);


		return optimizedState;
	}

	/**
   * Runs the module with the specified voltage while controlling to zero degrees. Must be called
   * periodically.
   */
  public void runCharacterization(double volts) {
    io_.setSteerVoltage(steerFeedback_.calculate(getAngle().getRadians(), 0.0));
    io_.setDriveVoltage(volts);
  }

	/** Disables all outputs to motors. */
	public void stop() {
		io_.setSteerVoltage(0.0);
		io_.setDriveVoltage(0.0);
	}

	/** Sets whether brake mode is enabled. */
	public void setBrakeMode(boolean enabled) {
		setBrakeMode(enabled, enabled);
	}

	/** Sets whether brake mode is enabled. */
	public void setBrakeMode(boolean drive, boolean steer) {
		io_.setDriveBrakeMode(drive);
		io_.setSteerBrakeMode(steer);
	}

	/** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return new Rotation2d(MathUtil.angleModulus(inputs_.steerAbsolutePositionRad));
  }

	/** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs_.drivePositionRad * wheelRadius_.get();
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs_.driveVelocityRadPerSec * wheelRadius_.get();
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs_.driveVelocityRadPerSec;
  }

  /** Returns the drive wheel radius. */
  public static double getWheelRadius() {
    return wheelRadius_.get();
  }
}
