package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    private TalonFX mClimberMotor;
    private DigitalInput mClimberTopSensor;
    private DigitalInput mClimberBottomSensor;
    private final static double LOWERSPEED = -0.1;
    private final static double RAISESPEED = 0.1;

    enum State{
        STOPPED,
        RAISING,
        LOWERING
    }
    private State currentState = State.STOPPED;


    public ClimberSubsystem() {
        mClimberMotor = new TalonFX(Constants.kClimberMotorId);
        mClimberTopSensor = new DigitalInput(Constants.kClimberTopSensorId);
        mClimberBottomSensor = new DigitalInput(Constants.kClimberBottomSensorId);
        mClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("ClimberTopSensor", isClimberAtTop());
        SmartDashboard.putBoolean("ClimberBottomSensor", isClimberAtBottom());
        if (currentState == State.RAISING && isClimberAtTop()) {
            stopMotors();   
        }
        else if(currentState == State.LOWERING && isClimberAtBottom()) {
            stopMotors();
        }
    }

    public boolean isClimberAtTop() {
        // Returns a boolean, true being that the climber is at farthest top it can be
        // not climber.get because it's inverted
        return !mClimberTopSensor.get();
    }

    public boolean isClimberAtBottom() {
        // Returns a boolean, true being that the climber is at farthest bottom it can be
        // not climber.get because it's inverted
        return !mClimberBottomSensor.get();

    }

    public void stopMotors() {
        mClimberMotor.set(0);
        currentState = State.STOPPED;
    }

    public void raiseMotors() {
        if (isClimberAtTop()) {
            stopMotors();
        }
        else { 
            mClimberMotor.set(RAISESPEED);
            currentState = State.RAISING;
        }     
    }
    
    public void lowerMotors() {
        if (isClimberAtBottom()) {
            stopMotors();
        }
        else {
            mClimberMotor.set(LOWERSPEED);
            currentState = State.LOWERING;
        }
    }
}
