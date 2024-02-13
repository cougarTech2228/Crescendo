package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
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
    private TalonSRX mBenderFeedMotor;
    double timeCheck;
    private enum ShooterState {
        EMPTY,
        ACQUIRINGBOTTOM,
        ACQUIRINGTOP,
        LOADED,
        FIRE_SPEAKER_PREP,
        FIRE_SPEAKER,
        BENDER_LOAD_INTERNAL_PREP,
        BENDER_LOAD_INTERNAL_LOAD_BENDER,
        BENDER_LOAD_INTERNAL_LOAD_BENDER_EXITING_MIDDLE,
        BENDER_LOAD_INTERNAL_LOADED,
        READY_FOR_FIRE_AMP,
        FIRE_AMP
    };
    private ShooterState shooterState = ShooterState.EMPTY;
    private boolean m_isLoaded = false;

    private enum BenderPosition {
        SHOOT_SPEAKER,
        SHOOT_AMP,
        LOAD_INTERNAL,
        LOAD_SOURCE
    };

    public enum OperatorEvent {
        NONE,
        PREP_AMP,
        FIRE_SPEAKER,
        FIRE_AMP,
    };
    private OperatorEvent currentEvent;

    // private CANSparkMax mBenderMotor;
    // private RelativeEncoder mBenderEncoder;
    // private SparkPIDController mBenderPidController;

    // private final static double BENDER_P = 0.0;
    // private final static double BENDER_I = 0.0;
    // private final static double BENDER_D = 0.0;

    private final static double LOAD_SPEED_GROUND = 1;
    private final static double LOAD_SPEED_BELT = -0.2;
    private final static double LOAD_SPEED_SHOOTER_FEED = 0.1;
    private final static double SHOOTER_DELAY = 0.5;
    private final static double SPEAKER_SHOOT_SPEED = 1.0;
    private final static double SPEAKER_FLYWHEEL_SHOOT_SPEED = 1.0;
    private final static double BENDER_SHOOT_SPEED = 1.0;
    private final static double LINEAR_ACTUATOR_RAISE_SPEED = 0.7;
    private final static double LINEAR_ACTUATOR_LOWER_SPEED = -0.7;
    private final static double BENDER_RAISE_SPEED = 0.2;
    private final static double BENDER_LOWER_SPEED = -0.2;

    /** angle where bender is out of the way so we can shoot at the speaker */
    private final static double BENDER_SPEAKER_LOCATION = 0;

    /** angle where bender is down so we can load it with a note from internal storage*/
    private final static double BENDER_INTERNAL_LOAD_NOTE_LOCATION = 0;

    /** angle where bender is in the correct location to shoot into the amp */
    private final static double BENDER_SHOOT_AMP_LOCATION = 0;

    /** distance away from expected location that we still concider good */
    private final static double BENDER_ANGLE_THRESHOLD = 0;

    enum ActuatorState {
            RAISING,
            LOWERING,
            STOPPED,
        }
    private ActuatorState currentActuatorState = ActuatorState.STOPPED;


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
        mLinearActuatorLeftMotor.follow(mLinearActuatorRightMotor);
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
        if(currentActuatorState == ActuatorState.RAISING && actuatorAtTop()) {
            stopLinearActuator();
        }
        else if(currentActuatorState == ActuatorState.LOWERING && actuatorAtBottom()) {
            stopLinearActuator();
        }
        switch (shooterState) {
            case EMPTY:
                stopAllShooterMotors();
                // TODO: if bender is not in default position, shooterState = ShooterState.SETBENDERTODEFAULTPOSITION;
                // else shooterState = ShooterState.
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
                stopAllShooterMotors();

                if (currentEvent == OperatorEvent.FIRE_SPEAKER) {
                    currentEvent = OperatorEvent.NONE;
                    changeState(ShooterState.FIRE_SPEAKER_PREP);
                } else if (currentEvent == OperatorEvent.PREP_AMP) {
                    currentEvent = OperatorEvent.NONE;
                    changeState(ShooterState.BENDER_LOAD_INTERNAL_PREP);
                }

                break;
            case FIRE_SPEAKER_PREP:
                mShooterFlywheelMotor.set(SPEAKER_FLYWHEEL_SHOOT_SPEED);
                // ensure bender is out of the way
                setBenderPosition(BenderPosition.SHOOT_SPEAKER);

                // TODO: Make this if statement work
                if (flywheelIsAtShootingSpeed() && benderIsInSpeakerLocation()) {
                    changeState(ShooterState.FIRE_SPEAKER);
                    timeCheck = Timer.getFPGATimestamp();
                }
                break;
            case FIRE_SPEAKER:
                launchSpeaker();
                if (isShooterDelayExpired()) {
                    m_isLoaded = false;
                    changeState(ShooterState.EMPTY);
                }
                break;

            case BENDER_LOAD_INTERNAL_PREP:
                setBenderPosition(BenderPosition.LOAD_INTERNAL);
                if (benderIsInInternalLoadingLocation()) {
                    changeState(ShooterState.BENDER_LOAD_INTERNAL_LOAD_BENDER);
                }
                break;
            case BENDER_LOAD_INTERNAL_LOAD_BENDER:
                mBenderTiltMotor.set(0);
                mBenderFeedMotor.set(ControlMode.PercentOutput, 0.05); // "slow"
                mShooterBeltMotor.set(0.05);
                mShooterFeedMotor.set(0.05);
                mShooterFlywheelMotor.set(0.05);
                if (isNoteAtTop()) {
                    changeState(ShooterState.BENDER_LOAD_INTERNAL_LOAD_BENDER_EXITING_MIDDLE);
                }
                break;
            case BENDER_LOAD_INTERNAL_LOAD_BENDER_EXITING_MIDDLE:
                if (!isNoteAtTop()) {
                    changeState(ShooterState.BENDER_LOAD_INTERNAL_LOADED);
                }
                break;
            case BENDER_LOAD_INTERNAL_LOADED:
                mBenderFeedMotor.set(ControlMode.PercentOutput, 0);
                mShooterBeltMotor.set(0);
                mShooterFeedMotor.set(0);
                mShooterFlywheelMotor.set(0);
                setBenderPosition(BenderPosition.SHOOT_AMP);
                if(benderIsInAmpLocation()) {
                    changeState(ShooterState.READY_FOR_FIRE_AMP);
                }
                break;
            case READY_FOR_FIRE_AMP:
                mBenderTiltMotor.set(0);
                // If amp shooter button is pressed
                // {timeCheck = Timer.getFPGATimestamp(); changeState(ShooterState.BENDERSHOOT;)}
                break;
            case FIRE_AMP:
                mBenderFeedMotor.set(ControlMode.PercentOutput, BENDER_SHOOT_SPEED);
                if (isShooterDelayExpired()) {
                    m_isLoaded = false;
                    changeState(ShooterState.EMPTY);
                }
                break;
        }
    }

    /**
     *  @return true if a note is at the front of the intake
     */
    private boolean isNoteAtBottom() {
        return !mGroundFeedSensor.get();
    }

    /**
     *  @return true if a note is currently seen between the ground
     * feeder, and the main chamber
     */
    private boolean isNoteAtMiddle() {
        return !mMiddleFeedSensor.get();
    }

    /**
     *  @return true if a note is currently seen between the main chamber and the bender
     */
    private boolean isNoteAtTop() {
        return !mTopFeedSensor.get();
    }

    /**
     * @return true if the actuator is at max high position
     */
    private boolean actuatorAtTop() {
        // 0.187241 is the highest angle
        return(mShooterAngleEncoder.get() <= 0.21);
    }

    /**
     * @return true if the actuator is at max bottom position.
     */
    private boolean actuatorAtBottom() {
        // 0.234322 is lowest angle.
        return(mShooterAngleEncoder.get() >= 0.222);
    }

    /**
     *  @return true if the bender is flipped back out of the way so 
     *  that a note can be shot at the speaker without hitting the bender
     */
    private boolean benderIsInSpeakerLocation() {
        // FIXME:
        // double deviation = Math.abs (mBenderTiltMotor.getAlternateEncoder(8192)
        //     .getPosition() - BENDER_SPEAKER_LOCATION);
        // return deviation < BENDER_ANGLE_THRESHOLD;
        return true;
    }

    /**
     * @return true if the bender is flipped forward so that a note can be fed from
     * the internal chamber into the bender
     */
    private boolean benderIsInInternalLoadingLocation() {
        // FIXME:
        //  double deviation = Math.abs (mBenderTiltMotor.getAlternateEncoder(8192)
        //      .getPosition() - BENDER_INTERNAL_LOAD_NOTE_LOCATION);
        //  return deviation < BENDER_ANGLE_THRESHOLD;
        return true;
    }

    /**
     * @return true if the bender is is the correct location for firing into the amp
     */
    private boolean benderIsInAmpLocation() {
        // FIXME:
        //  double deviation = Math.abs (mBenderTiltMotor.getAlternateEncoder(8192)
        //      .getPosition() - BENDER_SHOOT_AMP_LOCATION);
        //  return deviation < BENDER_ANGLE_THRESHOLD;
        return true;
    }

    /**
     * Sets the desired position of the bender
     */
    private void setBenderPosition(BenderPosition position) {
        System.out.println("setBenderPosition: " + position);
    }

    /**
     * @return true if the flywheel is up to speed for shooting in the speaker
     */
    private boolean flywheelIsAtShootingSpeed() {
        return mShooterFlywheelMotor.getVelocity().getValue() >= 1.0;
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

    public void raiseLinearActuator() {
        mLinearActuatorRightMotor.set(TalonSRXControlMode.PercentOutput, LINEAR_ACTUATOR_RAISE_SPEED);
        // mLinearActuatorLeftMotor.set(TalonSRXControlMode.PercentOutput, LINEAR_ACTUATOR_RAISE_SPEED);
        System.out.println("raising");
    }

    public void lowerLinearActuator() {
        mLinearActuatorRightMotor.set(TalonSRXControlMode.PercentOutput, LINEAR_ACTUATOR_LOWER_SPEED);
        // mLinearActuatorLeftMotor.set(TalonSRXControlMode.PercentOutput, LINEAR_ACTUATOR_LOWER_SPEED);
        System.out.println("lowering");
    }

    public void stopLinearActuator() {
        System.out.println("stop linear actuator shooter subsystem");
        mLinearActuatorRightMotor.set(TalonSRXControlMode.PercentOutput, 0);
        // mLinearActuatorLeftMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    private boolean isShooterDelayExpired() {
        // Returns a boolean, where true indicates that the time between the value of timeCheck and the present time is greater than SHOOTER_DELAY
        if ((Timer.getFPGATimestamp()-timeCheck)>SHOOTER_DELAY) {
            return true;
        } else {
            return false;
        }
    }

    private void acquiringBottom() {
        mGroundFeedMotor.set(LOAD_SPEED_GROUND);
        mShooterBeltMotor.set(LOAD_SPEED_BELT);
        mShooterFeedMotor.set(LOAD_SPEED_SHOOTER_FEED);

        // FIXME: remove this
        // mShooterFeedMotor.set(0.1);
        // mShooterFlywheelMotor.set(1);
    }

    private void launchSpeaker() {
        mShooterFeedMotor.set(SPEAKER_SHOOT_SPEED);
        mShooterBeltMotor.set(SPEAKER_SHOOT_SPEED);
    }

    public void stopAllShooterMotors() {
        // mBenderTiltMotor.set(0);
        mGroundFeedMotor.set(0);
        mShooterBeltMotor.set(0);
        mShooterFeedMotor.set(0);
        mShooterFlywheelMotor.set(0);
    }

    /**
     * This method is called to inform the ShooterSubsystem of operator inputs
     */
    public void operatorEvent(OperatorEvent event) {
        currentEvent = event;
    }
};