package frc.team2974.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.team2974.robot.RobotConstants.SwerveModuleK;
import frc.team2974.util.Conversions;

public class ModuleIOCTRE implements ModuleIO {
	private final WPI_TalonFX driveMotor_;
	private final WPI_TalonFX steerMotor_;
	private final CANCoder turnAbsEncoder_;
	private final Rotation2d absEncoderOffset_;

	public ModuleIOCTRE(int module) {
		var moduleConstants = SwerveModuleK.kModules[module];
		System.out.println("[ModuleIOCTRE][Ctor] Creating module " +
			Integer.toString(module) + "(" + moduleConstants.name() + ")");
		
		driveMotor_ = new WPI_TalonFX(moduleConstants.driveMotorId(), moduleConstants.canBus());
		steerMotor_ = new WPI_TalonFX(moduleConstants.steerMotorId(), moduleConstants.canBus());
		turnAbsEncoder_ = new CANCoder(moduleConstants.steerEncoderId(), moduleConstants.canBus());
		absEncoderOffset_ = moduleConstants.angleOffset();

		// TODO: configure devices
		driveMotor_.configFactoryDefault();
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		inputs.drivePositionRad =
			Conversions.falconToRadians(driveMotor_.getSelectedSensorPosition(), SwerveModuleK.kSteerRatio);
		inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
			Conversions.falconToRadPerSec(driveMotor_.getSelectedSensorVelocity(), SwerveModuleK.kSteerRatio));
		inputs.driveAppliedVolts = driveMotor_.getMotorOutputVoltage();
		inputs.driveCurrentAmps = new double[] {driveMotor_.getSupplyCurrent(), driveMotor_.getStatorCurrent()};
		inputs.driveTempCelcius = new double[] {driveMotor_.getTemperature()};

		inputs.steerPositionRad =
			Conversions.falconToRadians(steerMotor_.getSelectedSensorPosition(), SwerveModuleK.kSteerRatio);
		inputs.steerVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
			Conversions.falconToRadPerSec(steerMotor_.getSelectedSensorVelocity(), SwerveModuleK.kSteerRatio));
		inputs.steerAppliedVolts = steerMotor_.getMotorOutputVoltage();
		inputs.steerCurrentAmps = new double[] {steerMotor_.getSupplyCurrent(), steerMotor_.getStatorCurrent()};
		inputs.steerTempCelcius = new double[] {steerMotor_.getTemperature()};

		inputs.steerAbsolutePositionRad = MathUtil.angleModulus(
			 Rotation2d.fromDegrees(turnAbsEncoder_.getPosition())
			 .minus(absEncoderOffset_)
			 .getRadians()
		);
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveMotor_.setVoltage(volts);
	}

	@Override
	public void setSteerVoltage(double volts) {
		steerMotor_.setVoltage(volts);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		driveMotor_.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
	}

	// @Override
	// public void setOffboardSteerAngle(double positionCounts) {
	// 	steerMotor_.set(ControlMode.Position, positionCounts);
	// }

	// @Override
	// public void setOffboardDriveVelocity(double velocityCounts) {
	// 	driveMotor_.set(ControlMode.Velocity, velocityCounts);
	// }

	@Override
	public void setSteerBrakeMode(boolean enable) {
		steerMotor_.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
	}

	
}
