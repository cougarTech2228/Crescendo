package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    // private TalonFX mClimberMotor;
    // private DigitalInput mClimberTopSensor;
    // private DigitalInput mClimberBottomSensor;
    private final static double LOWER_VOLTAGE = -12;
    private final static double RAISE_VOLTAGE = 12;

    enum State {
        STOPPED,
        RAISING,
        LOWERING
    }

    @AutoLogOutput
    private State currentState = State.STOPPED;

    private final ClimberIO mIO;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private static ClimberSubsystem mInstance = null;

    public static ClimberSubsystem getInstance() {
        if (mInstance == null) {
            switch (Constants.currentMode) {
                case REAL:
                    mInstance = new ClimberSubsystem(new ClimberIOHW());
                    break;
                case SIM:
                    mInstance = new ClimberSubsystem(new ClimberIOSim());
                    break;
                default:
                    mInstance = new ClimberSubsystem(new ClimberIO(){});
                    break;
            }
        }
        return mInstance;
    }

    private ClimberSubsystem(ClimberIO io) {
        mIO = io;
        mIO.setNeutralMode(NeutralModeValue.Brake);

        // if (Robot.isDebug) {
        //     ShuffleboardTab m_sbTab = Shuffleboard.getTab("Climber (Debug)");

        //     m_sbTab.addBoolean("ClimberTopSensor", new BooleanSupplier() {
        //         @Override
        //         public boolean getAsBoolean() {
        //             return isClimberAtTop();
        //         };
        //     });

        //     m_sbTab.addBoolean("ClimberBottomSensor", new BooleanSupplier() {
        //         @Override
        //         public boolean getAsBoolean() {
        //             return isClimberAtBottom();
        //         };
        //     });

        //     m_sbTab.addDouble("Climber voltage", new DoubleSupplier() {
        //         @Override
        //         public double getAsDouble() {
        //             return mClimberMotor.getMotorVoltage().getValue();
        //         };
        //     });
        // }
    }

    @Override
    public void periodic() {
        mIO.updateInputs(inputs);
        Logger.processInputs("ClimberSubsystem", inputs);
        if (currentState == State.RAISING && isClimberAtTop()) {
            stopMotors();
        } else if (currentState == State.LOWERING && isClimberAtBottom()) {
            stopMotors();
        }
    }

    public boolean isClimberAtTop() {
        // Returns a boolean, true being that the climber is at farthest top it can be
        // not climber.get because it's inverted
        return !inputs.limitSwitchTop;
    }

    public boolean isClimberAtBottom() {
        // Returns a boolean, true being that the climber is at farthest bottom it can
        // be
        // not climber.get because it's inverted
        return !inputs.limitSwitchBottom;

    }

    public void stopMotors() {
        mIO.setClimberVoltage(0);
        currentState = State.STOPPED;
    }

    public void raiseMotors() {
        if (isClimberAtTop()) {
            stopMotors();
        } else {
            mIO.setClimberVoltage(RAISE_VOLTAGE);
            currentState = State.RAISING;
        }
    }

    public void lowerMotors() {
        if (isClimberAtBottom()) {
            stopMotors();
        } else {
            mIO.setClimberVoltage(LOWER_VOLTAGE);
            currentState = State.LOWERING;
        }
    }
}
