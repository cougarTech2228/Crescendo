package frc.robot.subsystems.shooterAngle;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ShooterAngleIOSim implements ShooterAngleIO {
    private final DutyCycleEncoder mShooterAngleEncoder = new DutyCycleEncoder(Constants.kShooterAngleEncoderId);
    private DCMotorSim angleMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 10, 0.0001);
    private double motorSimAppliedVolts = 0.0;

    @Override
    public void updateInputs(ShooterAngleIOInputs inputs) {
        inputs.currentVolts = motorSimAppliedVolts;
        inputs.currentAmps = angleMotorSim.getCurrentDrawAmps();
        inputs.currentAngle = mShooterAngleEncoder.getAbsolutePosition() * 1000;

        inputs.isShooterAtTop = (inputs.currentAngle < 374);
    }

    @Override
    public void setOutputPercentage(double percentage) {
        motorSimAppliedVolts = 12 * percentage;
        angleMotorSim.setInputVoltage(motorSimAppliedVolts);
    }
}