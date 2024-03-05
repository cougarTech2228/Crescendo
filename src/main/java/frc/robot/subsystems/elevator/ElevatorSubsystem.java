package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ElevatorSubsystem extends PIDSubsystem {

    private static final double ELEVATOR_HEIGHT_AMP = 65;
    private static final double ELEVATOR_HEIGHT_SOURCE = 80;
    private static final double ELEVATOR_THRESHOLD = 0.5;

    private static final double kP = 0.025;
    private static final double kI = 0.0;
    private static final double kD = 0.0001;
    private static final double kDt = 0.02;

    private static final double kMotorVoltageLimit = 0.7;

    private final ElevatorIO mIO;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    @AutoLogOutput
    private double encoderZeroValue = 0;

    private static final double ELEVATOR_UP_VOLTAGE = -5;
    private static final double ELEVATOR_DOWN_VOLTAGE = 5;
    private static final double ELEVATOR_HOME_VOLTAGE = 2;

    @AutoLogOutput
    private boolean mIsZeroed = false;

    enum State {
        STOPPED,
        RAISING,
        LOWERING,
    }

    @AutoLogOutput
    private State mCurrentState = State.STOPPED;

    public enum Position {
        HOME,
        AMP,
        SOURCE
    }

    @AutoLogOutput
    private Position mCurrentPosition = Position.HOME;

    private static final PIDController pidController = new PIDController(kP, kI, kD, kDt);

    private static ElevatorSubsystem mInstance = null;

    public static ElevatorSubsystem getInstance() {
        if (mInstance == null) {
            switch (Constants.currentMode) {
                case REAL:
                    mInstance = new ElevatorSubsystem(new ElevatorIOHW());
                    break;
                case SIM:
                    mInstance = new ElevatorSubsystem(new ElevatorIOSim());
                    break;
                default:
                    mInstance = new ElevatorSubsystem(new ElevatorIO(){});
                    break;
            }
        }
        return mInstance;
    }

    private ElevatorSubsystem(ElevatorIO io) {
        super(pidController, 0);
        mIO = io;

        pidController.setTolerance(ELEVATOR_THRESHOLD);

        mIO.setNeutralMode(NeutralModeValue.Brake);

        /*
         * if (Robot.isDebug) {
         * ShuffleboardTab sbTab = Shuffleboard.getTab("Elevator (Debug)");
         * 
         * sbTab.addString("Target position", new Supplier<String>() {
         * 
         * @Override
         * public String get() {
         * return mCurrentPosition.toString();
         * }
         * });
         * 
         * sbTab.addDouble("PID goal", new DoubleSupplier() {
         * 
         * @Override
         * public double getAsDouble() {
         * return m_controller.getSetpoint();
         * };
         * });
         * 
         * sbTab.addDouble("PID output", new DoubleSupplier() {
         * 
         * @Override
         * public double getAsDouble() {
         * return mElevatorMotor.getMotorVoltage().getValue();
         * };
         * });
         * 
         * sbTab.addDouble("Current Position:", new DoubleSupplier() {
         * 
         * @Override
         * public double getAsDouble() {
         * return inputs.elevatorPosition;
         * };
         * });
         * 
         * sbTab.addBoolean("ElevatorTopSensor", new BooleanSupplier() {
         * 
         * @Override
         * public boolean getAsBoolean() {
         * return mIO.isElevatorAtTop();
         * };
         * });
         * 
         * sbTab.addBoolean("ElevatorBottomSensor", new BooleanSupplier() {
         * 
         * @Override
         * public boolean getAsBoolean() {
         * return mIO.isElevatorAtBottom();
         * };
         * });
         * 
         * sbTab.addString("Elevator state", new Supplier<String>() {
         * 
         * @Override
         * public String get() {
         * return mCurrentState.name();
         * }
         * });
         * 
         * sbTab.addBoolean("PID Enabled", new BooleanSupplier() {
         * 
         * @Override
         * public boolean getAsBoolean() {
         * return isEnabled();
         * };
         * });
         * }
         */
    }

    private boolean isDisabled = DriverStation.isDisabled();

    @Override
    public void periodic() {
        mIO.updateInputs(inputs);
        Logger.processInputs("ElevatorSubsystem", inputs);
        super.periodic();

        Logger.recordOutput("ElevatorSubsystem/PID/setpoint", pidController.getSetpoint());
        Logger.recordOutput("ElevatorSubsystem/PID/positionError", pidController.getPositionError());
        Logger.recordOutput("ElevatorSubsystem/PID/atSetpoint", pidController.atSetpoint());
        Logger.recordOutput("ElevatorSubsystem/PID/enabled", isEnabled());


        if (DriverStation.isDisabled() != isDisabled) {
            isDisabled = DriverStation.isDisabled();
            if (isDisabled) {
                mIO.setNeutralMode(NeutralModeValue.Coast);
            } else {
                mIO.setNeutralMode(NeutralModeValue.Brake);
            }
        }

        if (isDisabled) {
            disable();
            stopMotor();
            return;
        }

        if (isElevatorAtBottom()) {
            // System.out.println("Elevator State " + mCurrentState);
            if (mCurrentState != State.RAISING) {
                stopMotor();
                setState(State.STOPPED);
                mCurrentPosition = Position.HOME;
            }
            if (mCurrentState == State.LOWERING) {
                System.out.println("disabling - lowering while home");
                disable();
            }
            if (!mIsZeroed) {
                mIsZeroed = true;
                encoderZeroValue = inputs.elevatorPosition;
            }
        }

        if (!mIsZeroed) {
            disable();
            mIO.setElevatorVoltage(ELEVATOR_HOME_VOLTAGE);
            setState(State.LOWERING);
            return;
        }

        if (mCurrentState == State.RAISING && isElevatorAtTop()) {
            stopMotor();
            setState(State.STOPPED);
        } else if (mCurrentState == State.LOWERING && isElevatorAtBottom()) {
            stopMotor();
            setState(State.STOPPED);
            mCurrentPosition = Position.HOME;
        }

        if (isElevatorAtBottom()) {
            ShooterSubsystem.isElevatorHome = true;
        } else {
            ShooterSubsystem.isElevatorHome = false;
        }
    }

    public void stopMotor() {
        // Stops elevator motor
        mIO.setElevatorVoltage(0);
        setState(State.STOPPED);
        // System.out.println("stopped");
    }

    public void raiseElevator() {
        disable();
        // setPosition(Position.AMP);
        // Moves the elevator
        System.out.println("called raise elevator");
        if (!isElevatorAtTop() && !ShooterSubsystem.isShooterLimit) {
            mIO.setElevatorVoltage(ELEVATOR_UP_VOLTAGE);
            // mElevatorMotor.set(TalonSRXControlMode.Velocity, 100);
            setState(State.RAISING);
            System.out.println("raising");
        }
    }

    public void lowerElevator() {
        disable();
        // setPosition(Position.HOME);
        // Moves the elevator down
        System.out.println("called lower elevator");
        if (!isElevatorAtBottom()) {
            mIO.setElevatorVoltage(ELEVATOR_DOWN_VOLTAGE);
            setState(State.LOWERING);
            System.out.println("lowering");
        }
    }

    public boolean isAtAmp() {
        return mCurrentPosition == Position.AMP && atGoal();
    }

    public boolean isAtHome() {
        return isElevatorAtBottom();
    }

    private boolean atGoal() {
        return pidController.atSetpoint();
    }

    protected double getMeasurement() {
        return inputs.elevatorPosition;
    }

    public void setPosition(Position position) {
        double target = 0;
        System.out.println("Elevator Set Position " + position);

        mCurrentPosition = position;
        if (position == Position.HOME) {
            target = encoderZeroValue;
        } else if (position == Position.AMP) {
            target = ELEVATOR_HEIGHT_AMP;
            target = encoderZeroValue - target;
            System.out.println("setting elevator height: " + target);
        } else if (position == Position.SOURCE) {
            target = ELEVATOR_HEIGHT_SOURCE;
            target = encoderZeroValue - target;
            System.out.println("Setting elevator height: " + target);
        }

        if (target < getMeasurement()) {
            System.out.println("CHANGE RAISING target: " + target + " , current: " + getMeasurement());

            setState(State.RAISING);
        } else if (target > getMeasurement()) {
            System.out.println("CHANGE LOWERING target: " + target + " , current: " + getMeasurement());
            setState(State.LOWERING);
        } else {
            setState(State.STOPPED);
        }

        pidController.setSetpoint(target);
        enable();
    }

    private void setState(State newState) {
        if (mCurrentState != newState) {
            System.out.println("Elevator State: " + mCurrentState + " --> " + newState);
            mCurrentState = newState;
        }
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        Logger.recordOutput("ElevatorSubsystem/PID/output", output);
        // clamp the output to a sane range
        double val;
        if (output < 0) {
            val = Math.max(-kMotorVoltageLimit, output);
        } else {
            val = Math.min(kMotorVoltageLimit, output);
        }
        if (val > 0 && isElevatorAtBottom()) {
            // don't go past bottom
            mIO.setElevatorVoltage(0);
        } else {
            // System.out.println("Elevator Use Output: " + val);
            mIO.setElevatorVoltage(12 * val);
        }
    }

    public boolean isElevatorAtBottom() {
        // Returns a boolean, opposite of elevator sensor.get
        // because it's inverted, false from sensor = there's something there
        // So we return the opposite, true means elevator at bottom
        return !inputs.limitSwitchBottom;
    }

    public boolean isElevatorAtTop() {
        // Returns a boolean, opposite of elevator sensor.get
        // because it's inverted, false from sensor = there's something there
        // So we return the opposite, true means elevator at top
        return !inputs.limitSwitchTop;
    }
}