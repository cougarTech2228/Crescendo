package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ShooterIOHW implements ShooterIO {

    private TalonFX mShooterFeedMotor = new TalonFX(Constants.kShooterFeedMotorId, Constants.kShooterFeedMotorBus);
    private TalonFX mShooterFlywheelMotor = new TalonFX(Constants.kShooterFlywheelMotorId, Constants.kShooterFlywheelMotorBus);
    private TalonSRX mBenderFeedMotor = new TalonSRX(Constants.kBenderFeedMotorId);

    private CANSparkMax mGroundFeedMotor = new CANSparkMax(Constants.kGroundFeedMotorId, MotorType.kBrushless);
    private CANSparkMax mShooterBeltMotor = new CANSparkMax(Constants.kShooterBeltMotorId, MotorType.kBrushless);

    private DigitalInput mGroundFeedSensor = new DigitalInput(Constants.kGroundFeedSensorId);
    private DigitalInput mMiddleFeedSensor = new DigitalInput(Constants.kMiddleFeedSensorId);
    private DigitalInput mTopFeedSensor = new DigitalInput(Constants.kTopFeedSensorId);


    private final StatusSignal<Double> flywheelMotorVelocity = mShooterFlywheelMotor.getVelocity();
    private final StatusSignal<Double> flywheelMotorAppliedVolts = mShooterFlywheelMotor.getMotorVoltage();
    private final StatusSignal<Double> flywheelMotorCurrentAmps = mShooterFlywheelMotor.getSupplyCurrent();

    private final StatusSignal<Double> feedMotorVelocity = mShooterFeedMotor.getVelocity();
    private final StatusSignal<Double> feedMotorAppliedVolts = mShooterFeedMotor.getMotorVoltage();
    private final StatusSignal<Double> feedMotorCurrentAmps = mShooterFeedMotor.getSupplyCurrent();

    public ShooterIOHW() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                flywheelMotorVelocity,
                flywheelMotorAppliedVolts,
                flywheelMotorCurrentAmps,
                feedMotorVelocity,
                feedMotorAppliedVolts,
                feedMotorCurrentAmps);
        mShooterFlywheelMotor.optimizeBusUtilization();
        mShooterFeedMotor.optimizeBusUtilization();
    }
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                flywheelMotorVelocity,
                flywheelMotorAppliedVolts,
                flywheelMotorCurrentAmps,
                feedMotorVelocity,
                feedMotorAppliedVolts,
                feedMotorCurrentAmps);
        inputs.isNoteAtBottom = !mGroundFeedSensor.get();
        inputs.isNoteAtMiddle = !mMiddleFeedSensor.get();
        inputs.isNoteAtTop = !mTopFeedSensor.get();

        inputs.groundFeedVelocity = mGroundFeedMotor.getEncoder().getVelocity();
        inputs.groundFeedAppliedVolts = mGroundFeedMotor.getAppliedOutput() * mGroundFeedMotor.getBusVoltage();
        inputs.groundFeedCurrentAmps = mGroundFeedMotor.getOutputCurrent();

        inputs.beltMotorVelocity = mShooterBeltMotor.getEncoder().getVelocity();
        inputs.beltMotorAppliedVolts = mShooterBeltMotor.getAppliedOutput() * mShooterBeltMotor.getBusVoltage();
        inputs.beltMotorCurrentAmps = mShooterBeltMotor.getOutputCurrent();

        inputs.flywheelMotorVelocity = flywheelMotorVelocity.getValueAsDouble();
        inputs.flywheelMotorAppliedVolts = flywheelMotorAppliedVolts.getValueAsDouble();
        inputs.flywheelMotorCurrentAmps = flywheelMotorCurrentAmps.getValueAsDouble();

        inputs.feedMotorVelocity = feedMotorVelocity.getValueAsDouble();
        inputs.feedMotorAppliedVolts = feedMotorAppliedVolts.getValueAsDouble();
        inputs.feedMotorCurrentAmps = feedMotorCurrentAmps.getValueAsDouble();

        inputs.benderFeedMotorAppliedVolts = mBenderFeedMotor.getMotorOutputVoltage();
        inputs.benderFeedMotorCurrentAmps = mBenderFeedMotor.getStatorCurrent();
    }

    @Override
    public void setGroundFeedMotorVoltage(double volts) {
        mGroundFeedMotor.setVoltage(volts);
    }

    @Override
    public void setBeltMotorVoltage(double volts) {
        mShooterBeltMotor.setVoltage(volts);
    }

    @Override
    public void setShooterFeedMotorVoltage(double volts) {
        mShooterFeedMotor.setVoltage(volts);
    }

    @Override
    public void setFlywheelMotorVoltage(double volts) {
        mShooterFlywheelMotor.setVoltage(volts);
    }

    @Override
    public void setBenderFeedMotorVoltage(double volts) {
        mBenderFeedMotor.set(TalonSRXControlMode.PercentOutput, volts / 12);
    }

    @Override
    public void setBrake(WhichMotor motor) {
        switch (motor) {
            case BELT:
                mShooterBeltMotor.setIdleMode(IdleMode.kBrake);
                break;
            case BENDER_FEED:
                mBenderFeedMotor.setNeutralMode(NeutralMode.Brake);
                break;
            case FEED_WHEEL:
                mShooterFeedMotor.setNeutralMode(NeutralModeValue.Brake);
                break;
            case FLYWHEEL:
                mShooterFlywheelMotor.setNeutralMode(NeutralModeValue.Brake);
                break;
            case GROUND_FEED:
                mGroundFeedMotor.setIdleMode(IdleMode.kBrake);
                break;
        }
    }

    @Override
    public void setCoast(WhichMotor motor) {
        switch (motor) {
            case BELT:
                mShooterBeltMotor.setIdleMode(IdleMode.kCoast);
                break;
            case BENDER_FEED:
                mBenderFeedMotor.setNeutralMode(NeutralMode.Coast);
                break;
            case FEED_WHEEL:
                mShooterFeedMotor.setNeutralMode(NeutralModeValue.Coast);
                break;
            case FLYWHEEL:
                mShooterFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
                break;
            case GROUND_FEED:
                mGroundFeedMotor.setIdleMode(IdleMode.kCoast);
                break;
        }
    }


    
}
