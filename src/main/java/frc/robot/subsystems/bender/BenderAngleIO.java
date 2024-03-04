package frc.robot.subsystems.bender;
import org.littletonrobotics.junction.AutoLog;

public interface BenderAngleIO {

    @AutoLog
    public static class BenderAngleIOInputs {
        public double benderAngle = 0.0;

        public double benderVoltage = 0.0;
        public double benderCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(BenderAngleIOInputs inputs) {}

    /** Run the motor at the specified voltage. */
    public default void setOutputPercentage(double percentage) {}

    public default void setBrakeMode() {}
    public default void setCoastMode() {}
}
