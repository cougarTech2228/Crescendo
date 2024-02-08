package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class ShooterSubsystem extends SubsystemBase {

    private TalonSRX mShooterFeedMotor;
    private TalonSRX mShooterFlywheelMotor;
    // private CANSparkMax mBenderMotor;
    // private RelativeEncoder mBenderEncoder;
    // private SparkPIDController mBenderPidController;

    // private final static double BENDER_P = 0.0;
    // private final static double BENDER_I = 0.0;
    // private final static double BENDER_D = 0.0;

    private final static double LOAD_SPEED = -0.4;
    public ShooterSubsystem() {
        mShooterFeedMotor = new TalonSRX(TunerConstants.kShooterFeedMotorId);
        mShooterFlywheelMotor = new TalonSRX(TunerConstants.kShooterFlywheelMotorId);
        // mBenderMotor = new CANSparkMax(TunerConstants.kBenderMotorId, MotorType.kBrushless);
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

        // SmartDashboard.putNumber("Alt Encoder Velocity", mBenderEncoder.getVelocity());
        // SmartDashboard.putNumber("Applied Output", mBenderMotor.getAppliedOutput());
    }

    public void test() {
        mShooterFlywheelMotor.set(ControlMode.PercentOutput, 1);
        mShooterFeedMotor.set(ControlMode.PercentOutput, 1);
    }

    public void startFlywheel() {
        mShooterFlywheelMotor.set(ControlMode.PercentOutput, 1);
    }

    public void feedNote() {
        mShooterFeedMotor.set(ControlMode.PercentOutput, 1);
    }

    public void stopMotors() {
        mShooterFeedMotor.set(ControlMode.PercentOutput, 0);
        mShooterFlywheelMotor.set(ControlMode.PercentOutput, 0);
    }

    public void loadNote() {
        mShooterFeedMotor.set(ControlMode.PercentOutput, LOAD_SPEED);
        mShooterFlywheelMotor.set(ControlMode.PercentOutput, LOAD_SPEED);
    }
};