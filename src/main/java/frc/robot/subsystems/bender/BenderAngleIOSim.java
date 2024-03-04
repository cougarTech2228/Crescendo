package frc.robot.subsystems.bender;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class BenderAngleIOSim implements BenderAngleIO {
    private DutyCycleEncoder m_benderAngleEncoder = new DutyCycleEncoder(Constants.kBenderAngleEncoderPin);
    private DCMotorSim mBenderTiltMotorSim = new DCMotorSim(DCMotor.getBag(1), 10, 0.0001);
    private double motorSimAppliedVolts = 0.0;

    public BenderAngleIOSim() {
        
    }

    @Override
    public void updateInputs(BenderAngleIOInputs inputs) {
        inputs.benderAngle = m_benderAngleEncoder.getAbsolutePosition() * 100;
        inputs.benderCurrentAmps = mBenderTiltMotorSim.getCurrentDrawAmps();
        inputs.benderVoltage = motorSimAppliedVolts;
    }

    @Override
    public void setOutputPercentage(double percentage) {
        motorSimAppliedVolts = 12 * percentage;
        mBenderTiltMotorSim.setInputVoltage(motorSimAppliedVolts);
    }

}