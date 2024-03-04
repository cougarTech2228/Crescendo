package frc.robot.subsystems.bender;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class BenderAngleIOHW implements BenderAngleIO {

    private TalonSRX mBenderTiltMotor = new TalonSRX(Constants.kBenderTiltMotorId);
    private DutyCycleEncoder m_benderAngleEncoder = new DutyCycleEncoder(Constants.kBenderAngleEncoderPin);

    public BenderAngleIOHW() {

    }

    public void updateInputs(BenderAngleIOInputs inputs) {
        inputs.benderAngle = m_benderAngleEncoder.getAbsolutePosition() * 100;
        inputs.benderCurrentAmps = mBenderTiltMotor.getStatorCurrent();
        inputs.benderVoltage = mBenderTiltMotor.getMotorOutputVoltage();
    }

    /** Run the motor at the specified voltage. */
    public void setOutputPercentage(double percentage) {
        mBenderTiltMotor.set(TalonSRXControlMode.PercentOutput, percentage);
    }

    public void setBrakeMode() {
        mBenderTiltMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setCoastMode() {
        mBenderTiltMotor.setNeutralMode(NeutralMode.Coast);
    }
}
