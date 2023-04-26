package frc.team2974.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.util.Units;
import frc.team2974.robot.RobotConstants;

public class GyroIOPigeon2 implements GyroIO {
	private final Pigeon2 pigeon;
	private final double[] yprDegrees = new double[3];
  private final double[] xyzDps = new double[3];

	public GyroIOPigeon2() {
		pigeon = new Pigeon2(RobotConstants.IdentityK.kPigeonCANID, RobotConstants.kDriveCANBus);

		pigeon.configFactoryDefault();
		pigeon.zeroGyroBiasNow();
		pigeon.setYaw(0.0);
		pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 20);
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		pigeon.getYawPitchRoll(yprDegrees);
    pigeon.getRawGyro(xyzDps);
    inputs.connected = pigeon.getLastError().equals(ErrorCode.OK);
    inputs.rollPositionRad = Units.degreesToRadians(yprDegrees[1]);
    inputs.pitchPositionRad = Units.degreesToRadians(-yprDegrees[2]);
    inputs.yawPositionRad = Units.degreesToRadians(yprDegrees[0]);
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(xyzDps[1]);
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-xyzDps[0]);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(xyzDps[2]);
	}
}
