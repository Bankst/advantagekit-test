// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2974.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class RobotConstants {
	public static final Mode currentMode = Robot.isReal() ?  Mode.REAL : Mode.SIM;

	public static enum Mode {
		/** Running on a real robot. */
		REAL,

		/** Running a physics simulator. */
		SIM,

		/** Replaying from a log file. */
		REPLAY
	}

	public static final String kRioCANBus = "";
	public static final String kDriveCANBus = "Canivore";
	public static final double kLoopPeriodSecs = 0.02;

	public static final class IdentityK {
		public static final int kPigeonCANID = 0;
	}

	public static final class SwerveK {
		public static final boolean kOnboardClosedLoop = !Robot.isReal() && true;

		public static final double kWheelDiameterInches = 3.875;
		public static final double kWheelDiameterMeters = Units.inchesToMeters(3.875);
		public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; 

		public static final double kMaxLinearSpeedFps = 14.5;
		public static final double kTrackWidthXInches = 27.125;
		public static final double kTrackWidthYInches = 27.125;
	}

	public static final class SwerveModuleK {
		public static record ModuleConstants (
			String name,
			int driveMotorId, int steerMotorId, int steerEncoderId, 
			Rotation2d angleOffset, String canBus
		) { }

		public static ModuleConstants[] kModules = {
			new ModuleConstants("FrontLeft", 2, 1, 1, Rotation2d.fromDegrees(3.95), kDriveCANBus),
			new ModuleConstants("FrontRight", 4, 3, 3, Rotation2d.fromDegrees(147.8), kDriveCANBus),
			new ModuleConstants("RearLeft", 8, 7, 7, Rotation2d.fromDegrees(73.4), kDriveCANBus),
			new ModuleConstants("RearRight", 6, 5, 5, Rotation2d.fromDegrees(42.36), kDriveCANBus)
		};

		public static final record SwerveMotorPIDConstants(
			double kP, double kD, double kS, double kV) {
				public SwerveMotorPIDConstants(double kP, double kD) {this(kP, kD, 0, 0);}
			}

		// constants used when using the on-Rio PID controls in simulation
		protected static final SwerveMotorPIDConstants kSimDrivePIDConstants = 
			new SwerveMotorPIDConstants(1, 0.0, 0.1, 0.2);
		protected static final SwerveMotorPIDConstants kSimSteerPIDConstants = 
			new SwerveMotorPIDConstants(1, 0);	

		// constants used when using the on-Rio PID controls on real hardware
		private static final SwerveMotorPIDConstants kOffboardDrivePIDConstants = 
			new SwerveMotorPIDConstants(0.3, 0, 0, 0);
		private static final SwerveMotorPIDConstants kOffboardSteerPIDConstants = 
			new SwerveMotorPIDConstants(23, 0);

		// constants used when using the on-TalonFX PID controls
		private static final SwerveMotorPIDConstants kOnboardDrivePIDConstants = 
			new SwerveMotorPIDConstants(0.9, 0.0, 0.18868, 0.12825);
		private static final SwerveMotorPIDConstants kOnboardSteerPIDConstants = 
			new SwerveMotorPIDConstants(10, 0);

		public static final SwerveMotorPIDConstants kDrivePIDConstants = kSimDrivePIDConstants;
			// !Robot.isReal() ? kSimDrivePIDConstants : // SwerveK.kOnboardClosedLoop ? 
				// kOnboardDrivePIDConstants; // : kOffboardDrivePIDConstants;

		public static final SwerveMotorPIDConstants kSteerPIDConstants = kSimSteerPIDConstants;
			// !Robot.isReal() ? kSimSteerPIDConstants : // SwerveK.kOnboardClosedLoop ? 
			// kOnboardSteerPIDConstants; // : kOffboardSteerPIDConstants;

		public static final SupplyCurrentLimitConfiguration steerSupplyLimit =
			new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);

		public static final TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();
		static {
			steerMotorConfig.slot0.kP = SwerveModuleK.kOffboardSteerPIDConstants.kP();
			steerMotorConfig.supplyCurrLimit = steerSupplyLimit;
		}
		public static boolean kSteerMotorInverted = true;
		public static double kSteerRatio = 150.0 / 7.0; // MK4i

		public static final SupplyCurrentLimitConfiguration driveSupplyLimit = 
			new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);

		public static final TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
		static {
			driveMotorConfig.slot0.kP = SwerveModuleK.kOffboardDrivePIDConstants.kP();
			driveMotorConfig.supplyCurrLimit = driveSupplyLimit;
			driveMotorConfig.openloopRamp = 0.25;
			driveMotorConfig.closedloopRamp = 0.0;
		}

		// (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
		public static double kDriveRatio = 6.75 / 1.0; // MK4i L2

		public static final CANCoderConfiguration steerEncoderConfig = new CANCoderConfiguration();
		static {
			steerEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
			steerEncoderConfig.sensorDirection = false;
			steerEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
			steerEncoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
		}
	}
}
