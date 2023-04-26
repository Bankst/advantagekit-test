package frc.team2974.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2974.robot.RobotConstants;
import frc.team6328.util.GeomUtil;
import frc.team6328.util.LoggedTunableNumber;

public class TeleDriveC extends CommandBase {
	public static final double matchEndThreshold =
			0.25; // FMS reports "0" ~250ms before the end of the match anyway
	public static final LoggedTunableNumber deadband =
			new LoggedTunableNumber("TeleDriveC/Deadband", 0.1);

	private final DriveS drive;
	private final Supplier<Double> leftXSupplier;
	private final Supplier<Double> leftYSupplier;
	private final Supplier<Double> rightYSupplier;
	private final Supplier<Boolean> robotRelativeOverride;

	private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

	public TeleDriveC(
		DriveS drive,
		Supplier<Double> leftXSupplier,
		Supplier<Double> leftYSupplier,
		Supplier<Double> rightYSupplier,
		Supplier<Boolean> robotRelativeOverride) {
		this.drive = drive;
		this.leftXSupplier = leftXSupplier;
		this.leftYSupplier = leftYSupplier;
		this.rightYSupplier = rightYSupplier;
		this.robotRelativeOverride = robotRelativeOverride;

		addRequirements(drive);
	}

	@Override
	public void initialize() {
		lastSpeeds = new ChassisSpeeds();
	}

	@Override
	public void execute() {
		// Go to X right before the end of the match
		// if (DriverStation.getMatchTime() >= 0.0 && DriverStation.getMatchTime() < matchEndThreshold) {
		// 	drive.stopWithX();
		// 	return;
		// }

		// TODO: define these elsewhere
		final double maxLinearAcceleration = 1000.0;
		final double maxAngularVelocity = 0.25;

		// Get values from double suppliers
		double leftX = leftXSupplier.get();
		double leftY = leftYSupplier.get();
		double rightY = rightYSupplier.get();

		// Get direction and magnitude of linear axes
		double linearMagnitude = Math.hypot(leftX, leftY);
		Rotation2d linearDirection = new Rotation2d(leftX, leftY);

		// Apply deadband
		linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband.get());
		rightY = MathUtil.applyDeadband(rightY, deadband.get());

		// Apply squaring
		linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
		rightY = Math.copySign(rightY * rightY, rightY);

		// Calcaulate new linear components
		Translation2d linearVelocity =
			new Pose2d(new Translation2d(), linearDirection)
				.transformBy(GeomUtil.translationToTransform(linearMagnitude, 0.0))
				.getTranslation();

		// Convert to meters per second
		ChassisSpeeds speeds =
			new ChassisSpeeds(
				linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
				linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
				rightY * maxAngularVelocity);

		// Convert from field relative
		if (!robotRelativeOverride.get()) {
			var driveRotation = drive.getRotation();
			if (DriverStation.getAlliance() == Alliance.Red) {
				driveRotation = driveRotation.plus(new Rotation2d(Math.PI));
			}
			speeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(
					speeds.vxMetersPerSecond,
					speeds.vyMetersPerSecond,
					speeds.omegaRadiansPerSecond,
					driveRotation);
		}

		speeds = new ChassisSpeeds(
			MathUtil.clamp(
				speeds.vxMetersPerSecond,
				lastSpeeds.vxMetersPerSecond - maxLinearAcceleration * RobotConstants.kLoopPeriodSecs,
				lastSpeeds.vxMetersPerSecond + maxLinearAcceleration * RobotConstants.kLoopPeriodSecs),
			MathUtil.clamp(
				speeds.vyMetersPerSecond,
				lastSpeeds.vyMetersPerSecond - maxLinearAcceleration * RobotConstants.kLoopPeriodSecs,
				lastSpeeds.vyMetersPerSecond + maxLinearAcceleration * RobotConstants.kLoopPeriodSecs),
			MathUtil.clamp(speeds.omegaRadiansPerSecond, -maxAngularVelocity, maxAngularVelocity));

		lastSpeeds = speeds;

		 // Send to drive
		//  var driveTranslation = AllianceFlipUtil.apply(drive.getPose().getTranslation());
		//  if (Math.abs(speeds.vxMetersPerSecond) < 1e-3
		// 		 && Math.abs(speeds.vyMetersPerSecond) < 1e-3
		// 		 && Math.abs(speeds.omegaRadiansPerSecond) < 1e-3
		// 		//  && driveTranslation.getX() > FieldConstants.Community.chargingStationInnerX
		// 		//  && driveTranslation.getX() < FieldConstants.Community.chargingStationOuterX
		// 		//  && driveTranslation.getY() > FieldConstants.Community.chargingStationRightY
		// 		//  && driveTranslation.getY() < FieldConstants.Community.chargingStationLeftY
		// 		) {
		// 	 drive.stopWithX();
		//  } else {
			 drive.runVelocity(speeds);
		//  }
	}

	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}
}
