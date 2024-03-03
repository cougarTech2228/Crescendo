package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants;

public class ClimberIOSim implements ClimberIO {
    private final TalonFX mClimberMotor = new TalonFX(Constants.kClimberMotorId, Constants.kClimberMotorBus);
    private DCMotorSim climberMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 100, 0.0001);
    private double climberMotorSimAppliedVolts = 0.0;
    private DIOSim topLimitSwitchSim = new DIOSim(new DigitalInput(Constants.kClimberTopSensorId));
    private DIOSim bottomLimitSwitchSim = new DIOSim(new DigitalInput(Constants.kClimberBottomSensorId));

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        climberMotorSim.update(0.02);

        inputs.climberPosition = climberMotorSim.getAngularPositionRotations();
        inputs.climberVelocity = climberMotorSim.getAngularVelocityRPM();
        inputs.climberAppliedVolts = climberMotorSimAppliedVolts;
        inputs.climberCurrentAmps = new double[] { climberMotorSim.getCurrentDrawAmps() };

        inputs.limitSwitchTop = topLimitSwitchSim.getValue();
        inputs.limitSwitchBottom = bottomLimitSwitchSim.getValue();

        mClimberMotor.getSimState().setRawRotorPosition(inputs.climberPosition);
        mClimberMotor.getSimState().setRotorVelocity(inputs.climberVelocity);
    }

    @Override
    public void setClimberVoltage(double volts) {
        climberMotorSimAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        climberMotorSim.setInputVoltage(climberMotorSimAppliedVolts);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
    }

}
