package frc.team2974.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.team2974.robot.RobotConstants;

public class ModuleIOSim implements ModuleIO {
	private final int index_;
	private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
	private FlywheelSim steerSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

	private double steerRelativePositionRad = 0.0;
	private double steerAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
	private double driveAppliedVolts = 3;
	private double steerAppliedVolts = 0.0;

	public ModuleIOSim(int index) {
		index_ = index;
		System.out.println("[Init] Creating ModuleIOSim"+Integer.toString(index));
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		driveSim.update(RobotConstants.kLoopPeriodSecs);
		steerSim.update(RobotConstants.kLoopPeriodSecs);

		double angleDiffRad = steerSim.getAngularVelocityRadPerSec() * RobotConstants.kLoopPeriodSecs;
		steerRelativePositionRad += angleDiffRad;
		steerAbsolutePositionRad += angleDiffRad;
		while (steerAbsolutePositionRad < 0) {
			steerAbsolutePositionRad += 2.0 * Math.PI;
		}
		while (steerAbsolutePositionRad > 2.0 * Math.PI) {
			steerAbsolutePositionRad -= 2.0 * Math.PI;
		}

		inputs.drivePositionRad =
			inputs.drivePositionRad
				+ (driveSim.getAngularVelocityRadPerSec() * RobotConstants.kLoopPeriodSecs);
		inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
		inputs.driveAppliedVolts = driveAppliedVolts;
		inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};
		inputs.driveTempCelcius = new double[] {};

		inputs.steerAbsolutePositionRad = steerAbsolutePositionRad;
		inputs.steerPositionRad = steerRelativePositionRad;
		inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
		inputs.steerAppliedVolts = steerAppliedVolts;
		inputs.steerCurrentAmps = new double[] {Math.abs(steerSim.getCurrentDrawAmps())};
		inputs.steerTempCelcius = new double[] {};
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveAppliedVolts = volts; //MathUtil.clamp(volts, -12.0, 12.0);
		if (volts > 0.01 || volts < -0.01) {
			System.out.println("[Module] got volts bitch!");
		}
		driveSim.setInputVoltage(driveAppliedVolts);
	}

	@Override
	public void setSteerVoltage(double volts) {
		steerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		steerSim.setInputVoltage(steerAppliedVolts);
	}
}
