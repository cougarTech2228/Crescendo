package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorIOHW implements ElevatorIO {
    private final TalonFX mElevatorMotor = new TalonFX(Constants.kElevatorMotorId, Constants.kElevatorMotorBus);
    private final DigitalInput mElevatorTopSensor = new DigitalInput(Constants.kElevatorTopSensorId);
    private final DigitalInput mElevatorBottomSensor = new DigitalInput(Constants.kElevatorBottomSensorId);

    private final StatusSignal<Double> elevatorPosition = mElevatorMotor.getPosition();
    private final StatusSignal<Double> elevatorVelocity = mElevatorMotor.getVelocity();
    private final StatusSignal<Double> elevatorAppliedVolts = mElevatorMotor.getMotorVoltage();
    private final StatusSignal<Double> elevatorCurrent = mElevatorMotor.getSupplyCurrent();


    public ElevatorIOHW() {
        var config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mElevatorMotor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                elevatorPosition,
                elevatorVelocity,
                elevatorAppliedVolts,
                elevatorCurrent);
        mElevatorMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                elevatorPosition,
                elevatorVelocity,
                elevatorAppliedVolts,
                elevatorCurrent);

        inputs.elevatorPosition = elevatorPosition.getValueAsDouble();
        inputs.elevatorVelocity = elevatorVelocity.getValueAsDouble();
        inputs.elevatorAppliedVolts = elevatorAppliedVolts.getValueAsDouble();
        inputs.elevatorCurrentAmps = new double[] { elevatorCurrent.getValueAsDouble() };

        inputs.limitSwitchTop = mElevatorTopSensor.get();
        inputs.limitSwitchBottom = mElevatorBottomSensor.get();
    }

    @Override
    public void setElevatorVoltage(double volts) {
        mElevatorMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        mElevatorMotor.setNeutralMode(neutralMode);
    }
}