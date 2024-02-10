package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    private TalonSRX mClimberMotor;
    private DigitalInput mClimberTopSensor;
    private DigitalInput mClimberBottomSensor;
    
    public ClimberSubsystem() {
        mClimberMotor = new TalonSRX(Constants.kClimberMotorId);
        mClimberTopSensor = new DigitalInput(Constants.kClimberTopSensorId);
        mClimberBottomSensor = new DigitalInput(Constants.kClimberBottomSensorId);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        SmartDashboard.putBoolean("ClimberTopSensor", isClimberAtTop());
        SmartDashboard.putBoolean("ClimberBottomSensor", isClimberAtBottom());

        // SmartDashboard.putNumber("Alt Encoder Velocity", mBenderEncoder.getVelocity());
        // SmartDashboard.putNumber("Applied Output", mBenderMotor.getAppliedOutput());
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
}
