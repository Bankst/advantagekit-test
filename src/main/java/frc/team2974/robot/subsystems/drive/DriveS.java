package frc.team2974.robot.subsystems.drive;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2974.robot.RobotConstants;
import frc.team2974.robot.RobotConstants.SwerveK;
import frc.team6328.util.LoggedTunableNumber;
import frc.team6328.util.PoseEstimator;
import frc.team6328.util.PoseEstimator.TimestampedVisionUpdate;

public class DriveS extends SubsystemBase {
	private static final double kCoastThresholdMetersPerSec =
			0.05; // Need to be under this to switch to coast when disabling
	private static final double kCoastThresholdSecs =
			6.0; // Need to be under the above speed for this length of time to switch to coast

	private final GyroIO gyroIO_;
	private final GyroIOInputsAutoLogged gyroInputs_ = new GyroIOInputsAutoLogged();
	private final Module[] modules_ = new Module[4]; // FL, FR, BL, BR;

	private static final LoggedTunableNumber maxLinearSpeed_ =
		new LoggedTunableNumber("Drive/MaxLinearSpeed", SwerveK.kMaxLinearSpeedFps);
	private static final LoggedTunableNumber trackWidthX_ =
		new LoggedTunableNumber("Drive/TrackWidthX", SwerveK.kTrackWidthXInches);
	private static final LoggedTunableNumber trackWidthY_ =
		new LoggedTunableNumber("Drive/TrackWidthY", SwerveK.kTrackWidthYInches);

	private double maxAngularSpeed_;
	private SwerveDriveKinematics kinematics_ = new SwerveDriveKinematics(getModuleTranslations());

	private boolean isCharacterizing_ = false;
	private ChassisSpeeds setpoint_ = new ChassisSpeeds();
	private SwerveModuleState[] lastSetpointStates_ =
		new SwerveModuleState[] {
			new SwerveModuleState(),
			new SwerveModuleState(),
			new SwerveModuleState(),
			new SwerveModuleState()
		};
	private double characterizationVolts_ = 0.0;
	private boolean isBrakeMode_ = false;
	private Timer lastMovementTimer_ = new Timer();

	private PoseEstimator poseEstimator = new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));
	private double[] lastModulePositionsMeters_ = new double[] {0.0, 0.0, 0.0, 0.0};
	private Rotation2d lastGyroYaw_ = new Rotation2d();
	private Twist2d fieldVelocity_ = new Twist2d();

	public DriveS(
		GyroIO gyroIO,
		ModuleIO flModuleIO,
		ModuleIO frModuleIO,
		ModuleIO blModuleIO,
		ModuleIO brModuleIO
	) {
		System.out.println("[DriveS][Ctor] Creating Drive Subsystem");
		gyroIO_ = gyroIO;
		modules_[0] = new Module(flModuleIO, 0);
		modules_[1] = new Module(flModuleIO, 1);
		modules_[2] = new Module(flModuleIO,2);
		modules_[3] = new Module(flModuleIO, 3);
		lastMovementTimer_.start();

		for (var module : modules_) {
			module.setBrakeMode(true);
		}
	}

	@Override
	public void periodic() {
		gyroIO_.updateInputs(gyroInputs_);
		Logger.getInstance().processInputs("Drive/Gyro", gyroInputs_);
		for (var module : modules_) {
			module.periodic();
		}

		// if (maxLinearSpeed_.hasChanged(maxLinearSpeed_.hashCode())
		// 		|| trackWidthX_.hasChanged(trackWidthX_.hashCode())
		// 		|| trackWidthY_.hasChanged(trackWidthY_.hashCode())) {
		// 	kinematics_ = new SwerveDriveKinematics(getModuleTranslations());
		// 	maxAngularSpeed_ = 
		// 		maxLinearSpeed_.get() / 
		// 			Arrays.stream(getModuleTranslations())
		// 				.map(translation -> translation.getNorm())
		// 				.max(Double::compare)
		// 				.get();
		// }

		// run modules
		if (DriverStation.isDisabled()) {
			// Stop moving while disabled
			for (var module : modules_) {
				module.stop();
			}

			// Clear setpoint logs
			Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
			Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});
		}  else if (isCharacterizing_) {
			// Run in characterization mode
			for (var module : modules_) {
				module.runCharacterization(characterizationVolts_);
			}

			// Clear setpoint logs
			Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
			Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});

		} else {
			// Calculate module setpoints
			var setpointTwist = new Pose2d().log(
				new Pose2d(
					setpoint_.vxMetersPerSecond * RobotConstants.kLoopPeriodSecs,
					setpoint_.vyMetersPerSecond * RobotConstants.kLoopPeriodSecs,
					new Rotation2d(setpoint_.omegaRadiansPerSecond * RobotConstants.kLoopPeriodSecs)));
			var adjustedSpeeds =
				new ChassisSpeeds(
					setpointTwist.dx / RobotConstants.kLoopPeriodSecs,
					setpointTwist.dy / RobotConstants.kLoopPeriodSecs,
					setpointTwist.dtheta / RobotConstants.kLoopPeriodSecs);
			SwerveModuleState[] setpointStates = kinematics_.toSwerveModuleStates(adjustedSpeeds);
			SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxLinearSpeed_.get());

			// Set to last angles if zero
			if (adjustedSpeeds.vxMetersPerSecond == 0.0
					&& adjustedSpeeds.vyMetersPerSecond == 0.0
					&& adjustedSpeeds.omegaRadiansPerSecond == 0) {
				for (int i = 0; i < 4; i++) {
					setpointStates[i] = new SwerveModuleState(0.0, lastSetpointStates_[i].angle);
				}
			}
			lastSetpointStates_ = setpointStates;

			// Send setpoints to modules
			SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
			for (int i = 0; i < 4; i++) {
				optimizedStates[i] = modules_[i].runSetpoint(setpointStates[i]);
			}

			// Log setpoint states
			Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);
			Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
		}

		// Log measured states
		SwerveModuleState[] measuredStates = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			measuredStates[i] = modules_[i].getState();
		}
		Logger.getInstance().recordOutput("SwerveStates/Measured", measuredStates);

		// Update odometry
		SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
			wheelDeltas[i] =
					new SwerveModulePosition(
							(modules_[i].getPositionMeters() - lastModulePositionsMeters_[i]),
							modules_[i].getAngle());
			lastModulePositionsMeters_[i] = modules_[i].getPositionMeters();
		}
		var twist = kinematics_.toTwist2d(wheelDeltas);
		var gyroYaw = new Rotation2d(gyroInputs_.yawPositionRad);
		if (gyroInputs_.connected) {
			twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw_).getRadians());
		}
		lastGyroYaw_ = gyroYaw;
		poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
		Logger.getInstance().recordOutput("Odometry/Robot", getPose());

		// Log 3D odometry pose
		Pose3d robotPose3d = new Pose3d(getPose());
		robotPose3d = robotPose3d.exp(new Twist3d(
			0.0, 0.0,
			Math.abs(gyroInputs_.pitchPositionRad) * trackWidthX_.get() / 2.0,
			0.0, gyroInputs_.pitchPositionRad, 0.0)
		).exp(new Twist3d(
			0.0, 0.0,
			Math.abs(gyroInputs_.rollPositionRad) * trackWidthY_.get() / 2.0,
			gyroInputs_.rollPositionRad, 0.0, 0.0)
		);
		Logger.getInstance().recordOutput("Odometry/Robot3d", robotPose3d);

		// Update field velocity
		ChassisSpeeds chassisSpeeds = kinematics_.toChassisSpeeds(measuredStates);
		Translation2d linearFieldVelocity =
				new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
						.rotateBy(getRotation());
		fieldVelocity_ =
			new Twist2d(
				linearFieldVelocity.getX(),
				linearFieldVelocity.getY(),
				gyroInputs_.connected
					? gyroInputs_.yawVelocityRadPerSec
					: chassisSpeeds.omegaRadiansPerSecond);

		// Update brake mode
		// boolean stillMoving = false;
		// for (int i = 0; i < 4; i++) {
		// 	if (Math.abs(modules_[i].getVelocityMetersPerSec()) > kCoastThresholdMetersPerSec) {
		// 		stillMoving = true;
		// 	}
		// }
		// if (stillMoving) lastMovementTimer_.reset();
		// if (DriverStation.isEnabled()) {
		// 	if (!isBrakeMode_) {
		// 		isBrakeMode_ = true;
		// 		for (var module : modules_) {
		// 			module.setBrakeMode(true);
		// 		}
		// 	}
		// } else {
		// 	if (isBrakeMode_ && lastMovementTimer_.hasElapsed(kCoastThresholdSecs)) {
		// 		isBrakeMode_ = false;
		// 		for (var module : modules_) {
		// 			module.setBrakeMode(false);
		// 		}
		// 	}
		// }
	}

	/**
	 * Runs the drive at the desired velocity.
	 *
	 * @param speeds Speeds in meters/sec
	 */
	public void runVelocity(ChassisSpeeds speeds) {
		isCharacterizing_ = false;
		setpoint_ = speeds;
	}

	/** Stops the drive. */
	public void stop() {
		runVelocity(new ChassisSpeeds());
	}

	/**
	 * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
	 * return to their normal orientations the next time a nonzero velocity is requested.
	 */
	public void stopWithX() {
		stop();
		for (int i = 0; i < 4; i++) {
			lastSetpointStates_[i] = new SwerveModuleState(
				lastSetpointStates_[i].speedMetersPerSecond, getModuleTranslations()[i].getAngle());
		}
	}

	/** Returns the maximum linear speed in meters per sec. */
	public double getMaxLinearSpeedMetersPerSec() {
		return Units.feetToMeters(maxLinearSpeed_.get());
	}

	/** Returns the maximum angular speed in radians per sec. */
	public double getMaxAngularSpeedRadPerSec() {
		return maxAngularSpeed_;
	}

	/**
	 * Returns the measured X, Y, and theta field velocities in meters per sec. The components of the
	 * twist are velocities and NOT changes in position.
	 */
	public Twist2d getFieldVelocity() {
		return fieldVelocity_;
	}

	/** Returns the current pitch (Y rotation). */
	public Rotation2d getPitch() {
		return new Rotation2d(gyroInputs_.pitchPositionRad);
	}

	/** Returns the current roll (X rotation). */
	public Rotation2d getRoll() {
		return new Rotation2d(gyroInputs_.rollPositionRad);
	}

	/** Returns the current yaw velocity (Z rotation) in radians per second. */
	public double getYawVelocity() {
		return gyroInputs_.yawVelocityRadPerSec;
	}

	/** Returns the current pitch velocity (Y rotation) in radians per second. */
	public double getPitchVelocity() {
		return gyroInputs_.pitchVelocityRadPerSec;
	}

	/** Returns the current roll velocity (X rotation) in radians per second. */
	public double getRollVelocity() {
		return gyroInputs_.rollVelocityRadPerSec;
	}

	/** Returns the current odometry pose. */
	public Pose2d getPose() {
		return poseEstimator.getLatestPose();
	}

	/** Returns the current odometry rotation. */
	public Rotation2d getRotation() {
		return poseEstimator.getLatestPose().getRotation();
	}

	/** Resets the current odometry pose. */
	public void setPose(Pose2d pose) {
		poseEstimator.resetPose(pose);
	}

	/** Adds vision data to the pose esimation. */
	public void addVisionData(List<TimestampedVisionUpdate> visionData) {
		poseEstimator.addVisionData(visionData);
	}

	/** Returns an array of module translations. */
	public Translation2d[] getModuleTranslations() {
		return new Translation2d[] {
			new Translation2d(trackWidthX_.get() / 2.0, trackWidthY_.get() / 2.0),
			new Translation2d(trackWidthX_.get() / 2.0, -trackWidthY_.get() / 2.0),
			new Translation2d(-trackWidthX_.get() / 2.0, trackWidthY_.get() / 2.0),
			new Translation2d(-trackWidthX_.get() / 2.0, -trackWidthY_.get() / 2.0)
		};
	}

	/** Runs forwards at the commanded voltage. */
	public void runCharacterizationVolts(double volts) {
		isCharacterizing_ = true;
		characterizationVolts_ = volts;
	}

	/** Returns the average drive velocity in radians/sec. */
	public double getCharacterizationVelocity() {
		double driveVelocityAverage = 0.0;
		for (var module : modules_) {
			driveVelocityAverage += module.getCharacterizationVelocity();
		}
		return driveVelocityAverage / 4.0;
	}
}
