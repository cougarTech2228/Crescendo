package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    
    private static final double ELEVATOR_HEIGHT_AMP = 65;
    private static final double ELEVATOR_HEIGHT_HOME = 20;



    private double encoderZeroValue = 0;



    private TalonFX mElevatorMotor;
    private DigitalInput mElevatorTopSensor;
    private DigitalInput mElevatorBottomSensor;
    private static final double ELEVATOR_SPEED_UP = -0.4;
    private static final double ELEVATOR_SPEED_DOWN = 0.4;
    private static final double ELEVATOR_SPEED_UP_FINE = -0.1;
    private static final double ELEVATOR_SPEED_DOWN_FINE= 0.1;

    private double mCurrentMeasurement = 0;
    private double mRequestedPosition = 0;
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

    private double mGoal = 0;
    private double AUTO_THRESHOLD = 1;
    private double AUTO_THRESHOLD_FINE = 10;
    private boolean mAutoEnabled = false;

    public ElevatorSubsystem() {
        

        mElevatorMotor = new TalonFX(Constants.kElevatorMotorId);
        mElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        mElevatorTopSensor = new DigitalInput(Constants.kElevatorTopSensorId);
        mElevatorBottomSensor = new DigitalInput(Constants.kElevatorBottomSensorId);
        ShuffleboardTab sbTab = Shuffleboard.getTab("Elevator (Debug)");

        sbTab.addDouble("Elevator Encoder", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return mCurrentMeasurement;
            };
        });
        sbTab.addDouble("Elevator Offset", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return encoderZeroValue;
            };
        });

        sbTab.addDouble("Elevator Requested Position", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return mRequestedPosition;
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

        sbTab.addBoolean("Elevator auto Enabled", new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return mAutoEnabled;
            };
        });

        sbTab.addDouble("Elevator goal", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return mGoal;
            };
        });
        sbTab.addString("Elevator state", new Supplier<String>() {
            @Override
            public String get() {
                return mCurrentState.name();
            }
        });
    }

    @Override
    public void periodic() {
        super.periodic();

        mCurrentMeasurement = mElevatorMotor.getRotorPosition().refresh().getValue();

        if (isElevatorAtBottom()) {
            encoderZeroValue = mElevatorMotor.getRotorPosition().getValue();
            mIsZeroed = true;
        }
    
        if(mCurrentState == State.RAISING && isElevatorAtTop()) {
            stopMotor();
            mAutoEnabled = false;
            mCurrentState = State.STOPPED;
        }
        else if(mCurrentState == State.LOWERING && isElevatorAtBottom()) {
            stopMotor();
            mAutoEnabled = false;
            mCurrentState = State.STOPPED;
        }

        if (mAutoEnabled && atGoal()) {
            stopMotor();
            mAutoEnabled = false;
        }

        if (!mIsZeroed) {
            mElevatorMotor.set(ELEVATOR_SPEED_DOWN);
            mCurrentState = State.LOWERING;
            return;
        }

        if (mAutoEnabled) {
            doAutoMovement();
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
        // setPosition(Position.AMP);
        //Moves the elevator
        System.out.println("called raise elevator");
        if(!isElevatorAtTop()) {
            mElevatorMotor.set(ELEVATOR_SPEED_UP);
            //mElevatorMotor.set(TalonSRXControlMode.Velocity, 100);
            mCurrentState = State.RAISING;
            System.out.println("raising");
        }
    }

    public void lowerElevator() {
        // setPosition(Position.HOME);
        //Moves the elevator down
        System.out.println("called lower elevator");
        if(!isElevatorAtBottom()) {
            mElevatorMotor.set(ELEVATOR_SPEED_DOWN);
            mCurrentState = State.LOWERING;
            System.out.println("lowering");
        }
    }

    public boolean isAtAmp() {
        return (mAutoEnabled == false && mCurrentPosition == Position.AMP);
    }

    public boolean isAtHome() {
        return isElevatorAtBottom();
    }

    private boolean atGoal() {
        return Math.abs(mGoal - getMeasurement()) < AUTO_THRESHOLD;
    }

    private void doAutoMovement() {
        if (atGoal()) {
            mAutoEnabled = false;
            return;
        }
        if (getMeasurement() > mGoal) {
            if(Math.abs(getMeasurement() - mGoal) < AUTO_THRESHOLD_FINE){
                mElevatorMotor.set(ELEVATOR_SPEED_UP_FINE);
            } else {
                mElevatorMotor.set(ELEVATOR_SPEED_UP);
            }
            mCurrentState = State.RAISING;
        }
        else if (getMeasurement() < mGoal) {
            if(Math.abs(getMeasurement() - mGoal) < AUTO_THRESHOLD_FINE){
                 mElevatorMotor.set(ELEVATOR_SPEED_DOWN_FINE);
            } else {
                mElevatorMotor.set(ELEVATOR_SPEED_DOWN);
            }
            mCurrentState = State.LOWERING;
        }
    }

    protected double getMeasurement() {
        return mCurrentMeasurement;
    }

    public void setPosition(Position position) {
        double target = 0;

        mCurrentPosition = position;
        if (position == Position.HOME) {
            if (!isElevatorAtBottom()) {
                mCurrentState = State.LOWERING;
                mElevatorMotor.set(ELEVATOR_SPEED_DOWN);
                return;
            } else {
                mCurrentState = State.STOPPED;
                stopMotor();
            }
        }
        else if (position == Position.AMP) {
            target = ELEVATOR_HEIGHT_AMP;
        
            target = encoderZeroValue - target;
            if (target > getMeasurement()) {
                mCurrentState = State.RAISING;
            } else if (target > getMeasurement()) {
                mCurrentState = State.LOWERING;
            } else {
                mCurrentState = State.STOPPED;
            }

            System.out.println("setting elevator height: " + target);
            mGoal = target;
            mAutoEnabled = true;
        }
    }
}