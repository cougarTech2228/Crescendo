package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    
    private TalonFX mElevatorMotor;
    private DigitalInput mElevatorTopSensor;
    private DigitalInput mElevatorBottomSensor;
    private static final double ELEVATOR_SPEED_UP = -0.6;
    private static final double ELEVATOR_SPEED_DOWN = 0.6;

    private double mRequestedPosition = 0;

    enum State {
        STOPPED,
        RAISING,
        LOWERING,
    }
    private State currentState = State.STOPPED;

    MotionMagicVoltage mMotionMagic = new MotionMagicVoltage(0);
    
    public ElevatorSubsystem() {
        mElevatorMotor = new TalonFX(Constants.kElevatorMotorId);
        mElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        mElevatorTopSensor = new DigitalInput(Constants.kElevatorTopSensorId);
        mElevatorBottomSensor = new DigitalInput(Constants.kElevatorBottomSensorId);
        ShuffleboardTab sbTab = Shuffleboard.getTab("Shooter (Debug)");

        sbTab.addDouble("Encoder", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return mElevatorMotor.getRotorPosition().getValueAsDouble();
            };
        });

        sbTab.addDouble("Requested Position", new DoubleSupplier() {
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


        TalonFXConfiguration cfg = new TalonFXConfiguration();

        /* Configure current limits */
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        mm.MotionMagicJerk = 50;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = 1;
        slot0.kI = 0;
        slot0.kD = 0.1;
        slot0.kV = 0.12;
        slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

        // FeedbackConfigs fdb = cfg.Feedback;
        // fdb.SensorToMechanismRatio = 12.8;
        mElevatorMotor.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic() {
        if(currentState == State.RAISING && isElevatorAtTop()) {
            stopMotor();
            mRequestedPosition = mElevatorMotor.getRotorPosition().getValueAsDouble();
            currentState = State.STOPPED;
        }
        else if(currentState == State.LOWERING && isElevatorAtBottom()) {
            stopMotor();
            mRequestedPosition = mElevatorMotor.getRotorPosition().getValueAsDouble();
            currentState = State.STOPPED;
        } else {
            mElevatorMotor.setControl(mMotionMagic.withPosition(mRequestedPosition));
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
        currentState = State.STOPPED;
        System.out.println("stopped");
    }

    public void raiseElevator() {
        // Moves the elevator
        System.out.println("called raise elevator");
        if(!isElevatorAtTop()) {
            mElevatorMotor.set(ELEVATOR_SPEED_UP);
            //mElevatorMotor.set(TalonSRXControlMode.Velocity, 100);
            currentState = State.RAISING;
            System.out.println("raising");
        }
    }

    public void lowerElevator() {
        // Moves the elevator down
        System.out.println("called lower elevator");
        if(!isElevatorAtBottom()) {
            mElevatorMotor.set(ELEVATOR_SPEED_DOWN);
            currentState = State.LOWERING;
            System.out.println("lowering");
        }
    }
}
