package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
    private DCMotorSim elevatorMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 10, 0.0001);
    private double elevatorMotorSimAppliedVolts = 0.0;
    private DIOSim topLimitSwitchSim = new DIOSim(new DigitalInput(Constants.kElevatorTopSensorId));
    private DIOSim bottomLimitSwitchSim = new DIOSim(new DigitalInput(Constants.kElevatorBottomSensorId));

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorMotorSim.update(0.02);

        inputs.elevatorPosition = elevatorMotorSim.getAngularPositionRotations();
        inputs.elevatorVelocity = elevatorMotorSim.getAngularVelocityRPM();
        inputs.elevatorAppliedVolts = elevatorMotorSimAppliedVolts;
        inputs.elevatorCurrentAmps = new double[] { elevatorMotorSim.getCurrentDrawAmps() };

        inputs.limitSwitchTop = topLimitSwitchSim.getValue();
        inputs.limitSwitchBottom = bottomLimitSwitchSim.getValue();
    }

    @Override
    public void setElevatorVoltage(double volts) {
        elevatorMotorSimAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        elevatorMotorSim.setInputVoltage(elevatorMotorSimAppliedVolts);
    }
}
