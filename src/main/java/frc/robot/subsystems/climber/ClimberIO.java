package frc.robot.subsystems.climber;
import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.signals.NeutralModeValue;


public interface ClimberIO {

    @AutoLog
    public static class ClimberIOInputs {
        public boolean limitSwitchTop = false;
        public boolean limitSwitchBottom = false;

        public double climberPosition = 0.0;
        public double climberVelocity = 0.0;
        public double climberAppliedVolts = 0.0;
        public double[] climberCurrentAmps = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimberIOInputs inputs) {}

    /** Run the climber motor at the specified voltage. */
    public default void setClimberVoltage(double volts) {}

    public default void setNeutralMode(NeutralModeValue neutralMode) {}
}
