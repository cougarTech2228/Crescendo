package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ClimberIOHW implements ClimberIO {
    private final TalonFX mElevatorMotor = new TalonFX(Constants.kClimberMotorId, Constants.kClimberMotorBus);
    private final DigitalInput mElevatorTopSensor = new DigitalInput(Constants.kClimberTopSensorId);
    private final DigitalInput mElevatorBottomSensor = new DigitalInput(Constants.kClimberBottomSensorId);

    private final StatusSignal<Double> climberPosition = mElevatorMotor.getPosition();
    private final StatusSignal<Double> climberVelocity = mElevatorMotor.getVelocity();
    private final StatusSignal<Double> climberAppliedVolts = mElevatorMotor.getMotorVoltage();
    private final StatusSignal<Double> climberCurrent = mElevatorMotor.getSupplyCurrent();


    public ClimberIOHW() {
        var config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mElevatorMotor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                climberPosition,
                climberVelocity,
                climberAppliedVolts,
                climberCurrent);
        mElevatorMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                climberPosition,
                climberVelocity,
                climberAppliedVolts,
                climberCurrent);

        inputs.climberPosition = climberPosition.getValueAsDouble();
        inputs.climberVelocity = climberVelocity.getValueAsDouble();
        inputs.climberAppliedVolts = climberAppliedVolts.getValueAsDouble();
        inputs.climberCurrentAmps = new double[] { climberCurrent.getValueAsDouble() };

        inputs.limitSwitchTop = mElevatorTopSensor.get();
        inputs.limitSwitchBottom = mElevatorBottomSensor.get();
    }

    @Override
    public void setClimberVoltage(double volts) {
        mElevatorMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        mElevatorMotor.setNeutralMode(neutralMode);
    }
}