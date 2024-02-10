package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ShooterSubsystem extends SubsystemBase {

    enum ShooterSubsystemState {
        EMPTY,
        LOADING_GROUND,
        LOADING_MIDDLE,
        LOADING_TOP,
        MAKE_BENDER_OPEN,
        SHOOT_BENDER,
        LOADED,
        SHOOTING_SPEAKER_PHASEONE,
        SHOOTING_SPEAKER_PHASETWO,
        PREPARING_AMP_PHASEONE,
        PREPARING_AMP_PHASETWO,
        FIRE_AMP,

    }
    private TalonFX mShooterFeedMotor;
    private TalonFX mShooterFlywheelMotor;
    private DigitalInput mGroundFeedSensor;
    private DigitalInput mMiddleFeedSensor;
    private DigitalInput mTopFeedSensor;
    private DutyCycleEncoder mShooterAngleEncoder;
    private CANSparkMax mGroundFeedMotor;
    private CANSparkMax mBenderTiltMotor;
    private CANSparkMax mShooterBeltMotor;
    private TalonFX mLinearActuatorLeftMotor;
    private TalonFX mLinearActuatorRightMotor;

    // private CANSparkMax mBenderMotor;
    // private RelativeEncoder mBenderEncoder;
    // private SparkPIDController mBenderPidController;

    // private final static double BENDER_P = 0.0;
    // private final static double BENDER_I = 0.0;
    // private final static double BENDER_D = 0.0;

    private final static double LOAD_SPEED = -0.2;
    private final static double POSITIVE_LOAD_SPEED = 0.2;

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
        mLinearActuatorLeftMotor = new TalonFX(Constants.kLinearActuatorLeftMotorId);
        mLinearActuatorRightMotor = new TalonFX(Constants.kLinearActuatorRightMotorId);

        mLinearActuatorLeftMotor.setControl(new Follower(Constants.kLinearActuatorRightMotorId, false));
        // mBenderMotor = new CANSparkMax(Constants.kBenderMotorId, MotorType.kBrushless);
        // mBenderEncoder = mBenderMotor.getAlternateEncoder(Type.kQuadrature, 8192); // REV Through-bore encoder is 8192 counts/rev
        // mBenderPidController = mBenderMotor.getPIDController();
        // mBenderPidController.setP(BENDER_P);
        // mBenderPidController.setI(BENDER_I);
        // mBenderPidController.setD(BENDER_D);
        // mBenderPidController.setFeedbackDevice(mBenderEncoder);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        SmartDashboard.putBoolean("GroundFeedSensor", isNoteAtBottom());
        SmartDashboard.putBoolean("MiddleFeedSensor", isNoteAtMiddle());
        SmartDashboard.putBoolean("TopFeedSensor", isNoteAtTop());
        SmartDashboard.putNumber("ShooterAngleEncoder", mShooterAngleEncoder.get());

        // SmartDashboard.putNumber("Alt Encoder Velocity", mBenderEncoder.getVelocity());
        // SmartDashboard.putNumber("Applied Output", mBenderMotor.getAppliedOutput());
    }

    public boolean isNoteAtBottom() {
        // Returns a boolean, true being that a note is at the 
        // bottom portion of the robot
        return mGroundFeedSensor.get();
    }

    public boolean isNoteAtMiddle() {
        // Returns a boolean, true being that a note is at
        // the middle portion of the robot
        return mMiddleFeedSensor.get();
    }

    public boolean isNoteAtTop() {
        // Returns a boolean, true being that a note is at
        // the top portion of the robot
        return mTopFeedSensor.get();
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
        mLinearActuatorRightMotor.set(0);
    }

    public void loadNote() {
        mShooterFeedMotor.set(LOAD_SPEED);
        mShooterFlywheelMotor.set(LOAD_SPEED);
    }

    public void loadNoteBottom() {
        // Loads a note that is at the bottom of the bot
        mGroundFeedMotor.set(POSITIVE_LOAD_SPEED);
    }

    public void startMiddleMotors() {
        // Loads a note that is in the middle of the bot
        mShooterBeltMotor.set(LOAD_SPEED);
        mShooterFeedMotor.set(POSITIVE_LOAD_SPEED);
        mShooterFlywheelMotor.set(POSITIVE_LOAD_SPEED);
    }
};