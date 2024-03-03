package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.signals.NeutralModeValue;


public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public boolean limitSwitchTop = false;
        public boolean limitSwitchBottom = false;

        public double elevatorPosition = 0.0;
        public double elevatorVelocity = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double[] elevatorCurrentAmps = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Run the elevator motor at the specified voltage. */
    public default void setElevatorVoltage(double volts) {}

    public default void setNeutralMode(NeutralModeValue neutralMode) {}
}
