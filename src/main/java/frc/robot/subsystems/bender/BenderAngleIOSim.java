package frc.robot.subsystems.bender;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class BenderAngleIOSim implements BenderAngleIO {
    private TalonSRX mBenderTiltMotor = new TalonSRX(Constants.kBenderTiltMotorId);
    private DutyCycleEncoder m_benderAngleEncoder = new DutyCycleEncoder(Constants.kBenderAngleEncoderPin);
    private DCMotorSim angleMotorSim = new DCMotorSim(DCMotor.getBag(1), 10, 0.0001);
    private double motorSimAppliedVolts = 0.0;

    public BenderAngleIOSim() {
        
    }

    @Override
    public void updateInputs(BenderAngleIOInputs inputs) {
        inputs.benderAngle = m_benderAngleEncoder.getAbsolutePosition() * 100;
        inputs.benderCurrentAmps = mBenderTiltMotor.getStatorCurrent();
        inputs.benderVoltage = mBenderTiltMotor.getMotorOutputVoltage();
    }

    @Override
    public void setOutputPercentage(double percentage) {
        motorSimAppliedVolts = 12 * percentage;
        angleMotorSim.setInputVoltage(motorSimAppliedVolts);
    }

}