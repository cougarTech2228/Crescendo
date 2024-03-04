package frc.robot.subsystems.shooterAngle;
import org.littletonrobotics.junction.AutoLog;


public interface ShooterAngleIO {

    @AutoLog
    public static class ShooterAngleIOInputs {
        public double currentVolts = 0.0;
        public double currentAmps = 0.0;
        public double currentAngle = 0.0;

        public boolean isShooterAtTop = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterAngleIOInputs inputs) {}

    /** Run the motor at the specified output percentage. */
    public default void setOutputPercentage(double percentage) {}

    public default void setBrakeMode() {}
}
