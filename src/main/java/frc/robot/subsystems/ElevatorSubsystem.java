package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ElevatorSubsystem extends PIDSubsystem {

    private static final double ELEVATOR_HEIGHT_AMP = 65;
    private static final double ELEVATOR_THRESHOLD = 0.5;

    private static final double kP = 0.025;
    private static final double kI = 0.0;
    private static final double kD = 0.0001;
    private static final double kDt = 0.02;

    private static final double kMotorVoltageLimit = 0.7;

    private double encoderZeroValue = 0;

    private TalonFX mElevatorMotor;
    private DigitalInput mElevatorTopSensor;
    private DigitalInput mElevatorBottomSensor;
    private static final double ELEVATOR_SPEED_UP = -0.4;
    private static final double ELEVATOR_SPEED_DOWN = 0.4;

    private double mCurrentMeasurement = 0;
    private boolean mIsZeroed = false;

    enum State {
        STOPPED,
        RAISING,
        LOWERING,
    }

    private State mCurrentState = State.STOPPED;

    enum Position {
        HOME,
        AMP
    }

    private Position mCurrentPosition = Position.HOME;

    private static final PIDController pidController = new PIDController(kP, kI, kD, kDt);

    private static ElevatorSubsystem mInstance = null;

    public static ElevatorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSubsystem();
        }
        return mInstance;
    }

    private ElevatorSubsystem() {
        super(pidController, 0);

        pidController.setTolerance(ELEVATOR_THRESHOLD);

        mElevatorMotor = new TalonFX(Constants.kElevatorMotorId);
        mElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        mElevatorTopSensor = new DigitalInput(Constants.kElevatorTopSensorId);
        mElevatorBottomSensor = new DigitalInput(Constants.kElevatorBottomSensorId);
        ShuffleboardTab sbTab = Shuffleboard.getTab("Elevator (Debug)");

        sbTab.addString("Target position", new Supplier<String>() {
            @Override
            public String get() {
                return mCurrentPosition.toString();
            }
        });

        sbTab.addDouble("PID goal", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return m_controller.getSetpoint();
            };
        });

        sbTab.addDouble("PID output", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return mElevatorMotor.getMotorVoltage().getValue();
            };
        });

        sbTab.addDouble("Current Angle:", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return mCurrentMeasurement;
            };
        });

        sbTab.addBoolean("ElevatorTopSensor", new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isElevatorAtTop();
            };
        });

        sbTab.addBoolean("ElevatorBottomSensor", new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isElevatorAtBottom();
            };
        });

        sbTab.addString("Elevator state", new Supplier<String>() {
            @Override
            public String get() {
                return mCurrentState.name();
            }
        });

        sbTab.addBoolean("PID Enabled", new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isEnabled();
            };
        });
    }

    private boolean isDisabled = DriverStation.isDisabled();

    @Override
    public void periodic() {
        super.periodic();

        if (isElevatorAtBottom()) {
            encoderZeroValue = mElevatorMotor.getRotorPosition().getValue();
            mIsZeroed = true;
        }

        if (!mIsZeroed) {
            mElevatorMotor.set(ELEVATOR_SPEED_DOWN);
            mCurrentState = State.LOWERING;
            return;
        }

        if (DriverStation.isDisabled() != isDisabled) {
            isDisabled = DriverStation.isDisabled();
            if (isDisabled) {
                mElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
            } else {
                mElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
            }
        }

        if (isDisabled) {
            disable();
            stopMotor();
            return;
        }

        mCurrentMeasurement = mElevatorMotor.getRotorPosition().refresh().getValue();

        if (mCurrentState == State.RAISING && isElevatorAtTop()) {
            stopMotor();
            mCurrentState = State.STOPPED;
        } else if (mCurrentState == State.LOWERING && isElevatorAtBottom()) {
            stopMotor();
            mCurrentState = State.STOPPED;
            mCurrentPosition = Position.HOME;
        }
    }

    public boolean isElevatorAtBottom() {
        // Returns a boolean, opposite of elevator sensor.get
        // because it's inverted, false from sensor = there's something there
        // So we return the opposite, true means elevator at bottom
        return !mElevatorBottomSensor.get();
    }

    public boolean isElevatorAtTop() {
        // Returns a boolean, opposite of elevator sensor.get
        // because it's inverted, false from sensor = there's something there
        // So we return the opposite, true means elevator at top
        return !mElevatorTopSensor.get();
    }

    public void stopMotor() {
        // Stops elevator motor
        mElevatorMotor.set(0);
        mCurrentState = State.STOPPED;
        // System.out.println("stopped");
    }

    public void raiseElevator() {
        disable();
        // setPosition(Position.AMP);
        // Moves the elevator
        System.out.println("called raise elevator");
        if (!isElevatorAtTop()) {
            mElevatorMotor.set(ELEVATOR_SPEED_UP);
            // mElevatorMotor.set(TalonSRXControlMode.Velocity, 100);
            mCurrentState = State.RAISING;
            System.out.println("raising");
        }
    }

    public void lowerElevator() {
        disable();
        // setPosition(Position.HOME);
        // Moves the elevator down
        System.out.println("called lower elevator");
        if (!isElevatorAtBottom()) {
            mElevatorMotor.set(ELEVATOR_SPEED_DOWN);
            mCurrentState = State.LOWERING;
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
        return mCurrentMeasurement;
    }

    public void setPosition(Position position) {
        double target = 0;

        mCurrentPosition = position;
        if (position == Position.HOME) {
            target = encoderZeroValue;
        } else if (position == Position.AMP) {
            target = ELEVATOR_HEIGHT_AMP;
            target = encoderZeroValue - target;
            System.out.println("setting elevator height: " + target);
        }

        if (target > getMeasurement()) {
            mCurrentState = State.RAISING;
        } else if (target > getMeasurement()) {
            mCurrentState = State.LOWERING;
        } else {
            mCurrentState = State.STOPPED;
        }

        pidController.setSetpoint(target);
        enable();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // clamp the output to a sane range
        double val;
        if (output < 0) {
            val = Math.max(-kMotorVoltageLimit, output);
        } else {
            val = Math.min(kMotorVoltageLimit, output);
        }
        mElevatorMotor.set(val);
    }
}