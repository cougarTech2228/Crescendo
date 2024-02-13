package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX mShooterFeedMotor;
    private TalonFX mShooterFlywheelMotor;
    private DigitalInput mGroundFeedSensor;
    private DigitalInput mMiddleFeedSensor;
    private DigitalInput mTopFeedSensor;
    private DutyCycleEncoder mShooterAngleEncoder;
    private CANSparkMax mGroundFeedMotor;
    private CANSparkMax mBenderTiltMotor;
    private CANSparkMax mShooterBeltMotor;
    private TalonSRX mLinearActuatorLeftMotor;
    private TalonSRX mLinearActuatorRightMotor;
    double timeCheck;
    private enum ShooterState {
        EMPTY,
        SETBENDERTODEFAULTPOSITION,
        BENDERISDEFAULT,
        ACQUIRINGBOTTOM,
        ACQUIRINGTOP,
        LOADED,
        FIRE1,
        FIRE2,
        PREP1,
        PREP2,
        PREP3,
        PREP4,
        PREP5,
        BENDERSHOOT
    };
    private ShooterState shooterState = ShooterState.EMPTY;
    private boolean m_isLoaded = false;

    // private CANSparkMax mBenderMotor;
    // private RelativeEncoder mBenderEncoder;
    // private SparkPIDController mBenderPidController;

    // private final static double BENDER_P = 0.0;
    // private final static double BENDER_I = 0.0;
    // private final static double BENDER_D = 0.0;

    private final static double LOAD_SPEED_GROUND = 1;
    private final static double LOAD_SPEED_BELT = -0.2;
    private final static double SHOOTER_DELAY = 0.5;
    private final static double SPEAKER_SHOOT_SPEED = 1.0;
    private final static double SPEAKER_FLYWHEEL_SHOOT_SPEED = 1.0;
    public ShooterSubsystem() {
        mShooterFeedMotor = new TalonFX(Constants.kShooterFeedMotorId);
        mShooterFlywheelMotor = new TalonFX(Constants.kShooterFlywheelMotorId);
        mGroundFeedSensor = new DigitalInput(Constants.kGroundFeedSensorId);
        mMiddleFeedSensor = new DigitalInput(Constants.kMiddleFeedSensorId);
        mTopFeedSensor = new DigitalInput(Constants.kTopFeedSensorId);
        mShooterAngleEncoder = new DutyCycleEncoder(Constants.kShooterAngleEncoderId);
        mGroundFeedMotor = new CANSparkMax(Constants.kGroundFeedMotorId, MotorType.kBrushless);
        mBenderTiltMotor = new CANSparkMax(Constants.kBenderTiltMotorId, MotorType.kBrushless);
        mShooterBeltMotor = new CANSparkMax(Constants.kShooterBeltMotorId, MotorType.kBrushless);
        mLinearActuatorLeftMotor = new TalonSRX(Constants.kLinearActuatorLeftMotorId);
        mLinearActuatorRightMotor = new TalonSRX(Constants.kLinearActuatorRightMotorId);

        // mLinearActuatorLeftMotor.setControl(new Follower(Constants.kLinearActuatorRightMotorId, false));
        // mBenderMotor = new CANSparkMax(Constants.kBenderMotorId, MotorType.kBrushless);
        // mBenderEncoder = mBenderMotor.getAlternateEncoder(Type.kQuadrature, 8192); // REV Through-bore encoder is 8192 counts/rev
        // mBenderPidController = mBenderMotor.getPIDController();
        // mBenderPidController.setP(BENDER_P);
        // mBenderPidController.setI(BENDER_I);
        // mBenderPidController.setD(BENDER_D);
        // mBenderPidController.setFeedbackDevice(mBenderEncoder);
    }
    
    public void initStateMachine(boolean preloaded) {
        if (preloaded) {
            changeState(ShooterState.LOADED);
        } else {
            changeState(ShooterState.EMPTY);
        }
    }

    private void changeState(ShooterState newState) {
        System.out.println("** Shooter State: " + shooterState + " --> " + newState);
        shooterState = newState;
    }

    public boolean isHoldingNote() {
        return m_isLoaded;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        SmartDashboard.putBoolean("GroundFeedSensor", isNoteAtBottom());
        SmartDashboard.putBoolean("MiddleFeedSensor", isNoteAtMiddle());
        SmartDashboard.putBoolean("TopFeedSensor", isNoteAtTop());
        SmartDashboard.putNumber("ShooterAngleEncoder", mShooterAngleEncoder.get());
        SmartDashboard.putString("Shooter State:", shooterState.name());

        // SmartDashboard.putNumber("Alt Encoder Velocity", mBenderEncoder.getVelocity());
        // SmartDashboard.putNumber("Applied Output", mBenderMotor.getAppliedOutput());
        switch (shooterState) {
            case EMPTY:
                stopAllShooterMotors();
                // TODO: if bender is not in default position, shooterState = ShooterState.SETBENDERTODEFAULTPOSITION;
                // else shooterState = ShooterState.

                if (isNoteAtBottom()) {
                    changeState(ShooterState.ACQUIRINGBOTTOM);
                }
                break;
            case SETBENDERTODEFAULTPOSITION:
                // TODO: rotate in correct direction until in correct position
                break;
            case BENDERISDEFAULT:
                if (isNoteAtBottom()) {
                    changeState(ShooterState.ACQUIRINGBOTTOM);
                }
                break;
            case ACQUIRINGBOTTOM:
                acquiringBottom();
                if (isNoteAtMiddle()) {
                    changeState(ShooterState.ACQUIRINGTOP);
                }
                break;
            case ACQUIRINGTOP:
                if (!isNoteAtMiddle()) {
                    m_isLoaded = true;
                    changeState(ShooterState.LOADED);
                }
                break;
            case LOADED:
                break;
            case FIRE1:
                mShooterFlywheelMotor.set(SPEAKER_FLYWHEEL_SHOOT_SPEED);
                // // TODO: Make this if statement work
                // if (/*current flywheel speed is suffiently fast*/) {
                //     changeState(ShooterState.FIRE2);
                //     timeCheck = Timer.getFPGATimestamp();
                // }
                break;
            case FIRE2:
                launchSpeaker();
                if (isShooterDelayExpired()) {
                    m_isLoaded = false;
                    changeState(ShooterState.EMPTY);
                }
                break;
            case PREP1:
                // TODO: Bender rotation motor on
                break;
            case PREP2:
                mBenderTiltMotor.set(0);
                break;
            case PREP3:
                break;
            case PREP4:
                break;
            case PREP5:
                break;
            case BENDERSHOOT:
                break;
        }
    }

    public boolean isNoteAtBottom() {
        // Returns a boolean, true being that a note is at the 
        // bottom portion of the robot
        return !mGroundFeedSensor.get();
    }

    public boolean isNoteAtMiddle() {
        // Returns a boolean, true being that a note is at
        // the middle portion of the robot
        return !mMiddleFeedSensor.get();
    }

    public boolean isNoteAtTop() {
        // Returns a boolean, true being that a note is at
        // the top portion of the robot
        return !mTopFeedSensor.get();
    }

    public void test() {
        mShooterFlywheelMotor.set(1);
        mShooterFeedMotor.set(1);
    }

    public void startFlywheel() {
        mShooterFlywheelMotor.set(1);
    }

    public void feedNote() {
        mShooterFeedMotor.set(1);
    }

    public void stopFeedShootMotors() {
        mShooterFeedMotor.set(0);
        mShooterFlywheelMotor.set(0);
        mGroundFeedMotor.set(0);
        mShooterBeltMotor.set(0);
        mBenderTiltMotor.set(0);
    }

    public void stopBottomMotor() {
        mGroundFeedMotor.set(0);
    }

    public void stopLinearActuator() {
        mLinearActuatorRightMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
    
    public void stopMotors() {
        mShooterFeedMotor.set(0);
        mShooterFlywheelMotor.set(0);
    }

    public void loadNote() {
        // mShooterFeedMotor.set(LOAD_SPEED);
        // mShooterFlywheelMotor.set(LOAD_SPEED);
    }

    public boolean isShooterDelayExpired() {
        if ((Timer.getFPGATimestamp()-timeCheck)>SHOOTER_DELAY) {
            return true;
        } else {
            return false;
        }
    }
    public void acquiringBottom() {
        mGroundFeedMotor.set(LOAD_SPEED_GROUND);
        mShooterBeltMotor.set(LOAD_SPEED_BELT);

        // FIXME: remove this
        mShooterFeedMotor.set(0.1);
        mShooterFlywheelMotor.set(1);
    }
    public void launchSpeaker() {
        mShooterFeedMotor.set(SPEAKER_SHOOT_SPEED);
        mShooterBeltMotor.set(SPEAKER_SHOOT_SPEED);
    }
    public void stopAllShooterMotors() {
        mBenderTiltMotor.set(0);
        mGroundFeedMotor.set(0);
        mShooterBeltMotor.set(0);
        mShooterFeedMotor.set(0);
        mShooterFlywheelMotor.set(0);
        mLinearActuatorLeftMotor.set(ControlMode.PercentOutput, 0);
        mLinearActuatorRightMotor.set(ControlMode.PercentOutput, 0);

    }
};