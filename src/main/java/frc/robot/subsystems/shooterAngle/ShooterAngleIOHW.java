package frc.robot.subsystems.shooterAngle;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ShooterAngleIOHW implements ShooterAngleIO {
    private final TalonSRX mLinearActuatorMotor = new TalonSRX(Constants.kLinearActuatorMotorId);
    private final DutyCycleEncoder mShooterAngleEncoder = new DutyCycleEncoder(Constants.kShooterAngleEncoderId);

    public ShooterAngleIOHW() {
    }

    @Override
    public void updateInputs(ShooterAngleIOInputs inputs) {
        inputs.currentVolts = mLinearActuatorMotor.getMotorOutputVoltage();
        inputs.currentAmps = mLinearActuatorMotor.getStatorCurrent();
        inputs.currentAngle = mShooterAngleEncoder.getAbsolutePosition() * 1000;

        inputs.isShooterAtTop = (inputs.currentAngle < ShooterAngleSubsystem.SHOOTER_AT_TOP);
    }

    @Override
    public void setOutputPercentage(double percentage) {
        mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, percentage);
    }

    @Override
    public void setBrakeMode() {
        mLinearActuatorMotor.setNeutralMode(NeutralMode.Brake);
    }
}