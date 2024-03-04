package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {

    private DCMotorSim mShooterFeedMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.0001);
    private DCMotorSim mShooterFlywheelMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.0001);
    private DCMotorSim mBenderFeedMotorSim = new DCMotorSim(DCMotor.getBag(1), 1, 0.0001);
    private DCMotorSim mGroundFeedMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.0001);
    private DCMotorSim mShooterBeltMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.0001);

    private DIOSim mGroundFeedSensorSim = new DIOSim(new DigitalInput(Constants.kGroundFeedSensorId));
    private DIOSim mMiddleFeedSensorSim = new DIOSim(new DigitalInput(Constants.kMiddleFeedSensorId));
    private DIOSim mTopFeedSensorSim = new DIOSim(new DigitalInput(Constants.kTopFeedSensorId));

    private double groundFeedAppliedVolts = 0.0;
    private double beltMotorAppliedVolts = 0.0;
    private double flywheelMotorAppliedVolts = 0.0;
    private double feedMotorAppliedVolts = 0.0;
    private double benderFeedMotorAppliedVolts = 0.0;
    
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.isNoteAtBottom = !mGroundFeedSensorSim.getValue();
        inputs.isNoteAtMiddle = !mMiddleFeedSensorSim.getValue();
        inputs.isNoteAtTop = !mTopFeedSensorSim.getValue();

        inputs.groundFeedVelocity = mGroundFeedMotorSim.getAngularVelocityRPM();
        inputs.groundFeedAppliedVolts = groundFeedAppliedVolts;
        inputs.groundFeedCurrentAmps = mGroundFeedMotorSim.getCurrentDrawAmps();

        inputs.beltMotorVelocity = mShooterBeltMotorSim.getAngularVelocityRPM();
        inputs.beltMotorAppliedVolts = beltMotorAppliedVolts;
        inputs.beltMotorCurrentAmps = mShooterBeltMotorSim.getCurrentDrawAmps();

        inputs.flywheelMotorVelocity = mShooterFlywheelMotorSim.getAngularVelocityRPM();
        inputs.flywheelMotorAppliedVolts = flywheelMotorAppliedVolts;
        inputs.flywheelMotorCurrentAmps = mShooterFlywheelMotorSim.getCurrentDrawAmps();

        inputs.feedMotorVelocity = mShooterFeedMotorSim.getAngularVelocityRPM();
        inputs.feedMotorAppliedVolts = feedMotorAppliedVolts;
        inputs.feedMotorCurrentAmps = mShooterFeedMotorSim.getCurrentDrawAmps();

        inputs.benderFeedMotorAppliedVolts = benderFeedMotorAppliedVolts;
        inputs.benderFeedMotorCurrentAmps = mBenderFeedMotorSim.getCurrentDrawAmps();
    }

    @Override
    public void setGroundFeedMotorVoltage(double volts) {
        mGroundFeedMotorSim.setInputVoltage(volts);
        groundFeedAppliedVolts = volts;
    }

    @Override
    public void setBeltMotorVoltage(double volts) {
        mShooterBeltMotorSim.setInputVoltage(volts);
        beltMotorAppliedVolts = volts;
    }

    @Override
    public void setShooterFeedMotorVoltage(double volts) {
        mShooterFeedMotorSim.setInputVoltage(volts);
        feedMotorAppliedVolts = volts;
    }

    @Override
    public void setFlywheelMotorVoltage(double volts) {
        mShooterFlywheelMotorSim.setInputVoltage(volts);
        flywheelMotorAppliedVolts = volts;
    }

    @Override
    public void setBenderFeedMotorVoltage(double volts) {
        mBenderFeedMotorSim.setInputVoltage(volts);
        benderFeedMotorAppliedVolts = volts;
    }
    
}
