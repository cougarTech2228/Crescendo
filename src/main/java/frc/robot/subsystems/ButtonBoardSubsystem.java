package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem.ShooterPosition;
import frc.robot.subsystems.ShooterSubsystem.OperatorEvent;

public class ButtonBoardSubsystem extends SubsystemBase {
    private ClimberSubsystem climberSubsystem;

    private ShooterSubsystem shooterSubsystem;

    private double m_fwdRevJoystick;
    private double m_strafeJoystick;

    private boolean m_strafeReset = true;
    private boolean m_fwdRevReset = true;

    public static final double FINE_STRAFE_DISTANCE_CM = 3.0;
    private static final double STRAFE_SPEED = 0.1;

    private static final double FINE_DRIVE_DISTANCE_CM = 5.0;
    private static final double DRIVE_SPEED = 0.1;

    private static final double FINE_TURN_DEGREES = 2.0;
    private static final double ANGULAR_VELOCITY = 0.4;

    private enum ButtonBoardOperationMode {
        Drive,
        Camera
    }
    private ShooterSubsystem m_shooterSubsystem;


    private final int kJoystickChannel1 = 1;
    private final int kJoystickChannel2 = 2;

    private Joystick m_joystick1;
    private Joystick m_joystick2;

    private ButtonBoardOperationMode m_operationMode;

    public ButtonBoardSubsystem(ClimberSubsystem climberSubsystem, ShooterSubsystem shooterSubsystem) {
        m_joystick1 = new Joystick(kJoystickChannel1);
        m_joystick2 = new Joystick(kJoystickChannel2);
        this.climberSubsystem = climberSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }
    

    // Joystick #1 Buttons

    private JoystickButton lowerElevatorButton() {
        return new JoystickButton(m_joystick2, 5);
    }

    private JoystickButton raiseElevatorButton() {
        return new JoystickButton(m_joystick2, 3);
    }

    private JoystickButton raiseClimberButton() {
        return new JoystickButton(m_joystick2, 4);
    }
    
    private JoystickButton lowerClimberButton() {
        return new JoystickButton(m_joystick2, 6);
    }

    private JoystickButton raiseLinearActuatorButton() {
        return new JoystickButton(m_joystick2, 2);
    }

    private JoystickButton lowerLinearActuatorButton() {
        return new JoystickButton(m_joystick2, 1);
    }

    private JoystickButton spitButton() {
        return new JoystickButton(m_joystick2, 8);
    }

    private JoystickButton driveCameraSwitch() {
        return new JoystickButton(m_joystick2, 7);
    }    

    // Joystick #2 Buttons
    

    private JoystickButton prepSpeakerFrontButton() {
        return new JoystickButton(m_joystick1, 1);
    }

    private JoystickButton prepSpeakerSideButton() {
        return new JoystickButton(m_joystick1, 2);
    }

    private JoystickButton prepAmpButton() {
        return new JoystickButton(m_joystick1, 3);
    }

    private JoystickButton shootButton() {
        return new JoystickButton(m_joystick1, 4);
    }

    private JoystickButton blank5() {
        return new JoystickButton(m_joystick1, 5);
    }  

    private JoystickButton blank6() {
        return new JoystickButton(m_joystick1, 6);
    }

    private JoystickButton blank7() {
        return new JoystickButton(m_joystick1, 7);
    }

    private JoystickButton blank8() {
        return new JoystickButton(m_joystick1, 8);
    }

    public boolean isDriveOperationMode() {
        return (m_operationMode == ButtonBoardOperationMode.Drive);
    }

    private void setOperationMode() {
        if (driveCameraSwitch().getAsBoolean()) {
            m_operationMode = ButtonBoardOperationMode.Drive;
        } else {
            m_operationMode = ButtonBoardOperationMode.Camera;
        }
        System.out.println(m_operationMode);
    }

    @Override
    public void periodic() {
    }

    public double getJoystickX(){
        return m_joystick2.getX();
    }

    public double getJoystickY(){
        return m_joystick2.getY();
    }

    public void configureButtonBindings() {

        driveCameraSwitch().onTrue(
            new InstantCommand(() -> setOperationMode())
        );

        driveCameraSwitch().onFalse(
            new InstantCommand(() -> setOperationMode())
        );

        raiseElevatorButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("raiseElevatorButton Press");
                shooterSubsystem.raiseElevator();
            })
        );

        raiseElevatorButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("raiseElevatorButton Release");
                shooterSubsystem.stopElevator();
            })
        );
        
        lowerElevatorButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("lowerElevatorButton Press");
                shooterSubsystem.lowerElevator();
            })
        );

        lowerElevatorButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("lowerElevatorButton Release");
                shooterSubsystem.stopElevator();
            })
        );

        raiseClimberButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("raiseClimberButton Press");
                climberSubsystem.raiseMotors();
            })
        );

        raiseClimberButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("raiseClimberButton Release");
                climberSubsystem.stopMotors();
            })
        );

        lowerClimberButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("lowerClimberButton Press");
                climberSubsystem.lowerMotors();
            })
        );

        lowerClimberButton().onFalse(
            new InstantCommand(() -> {
                climberSubsystem.stopMotors();
                System.out.println("lowerClimberButton Release");
            })
        );  

        raiseLinearActuatorButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("raiseLinearActuatorButton Press");
                shooterSubsystem.raiseLinearActuator();
            })
        );

        raiseLinearActuatorButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("raiseLinearActuatorButton Release");
                shooterSubsystem.stopLinearActuator();
            })
        );

        lowerLinearActuatorButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("lowerLinearActuatorButton Press");
                shooterSubsystem.lowerLinearActuator();
            })
        );

        lowerLinearActuatorButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("lowerLinearActuatorButton Release");
                shooterSubsystem.stopLinearActuator();
            })
        );

        spitButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("Spit Button Pressed");
                shooterSubsystem.operatorEvent(OperatorEvent.SPIT);
            })
        ); 

        spitButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("Spit Button Released");
                shooterSubsystem.operatorEvent(OperatorEvent.NONE);
            })  
        );

        prepSpeakerFrontButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("Prep Speaker Front Button Pressed");
                shooterSubsystem.operatorEvent(OperatorEvent.PREP_SPEAKER_FRONT);
            })
        );

        blank7().onTrue(
            new InstantCommand(() -> {
                System.out.println("blank7 Pressed");
                shooterSubsystem.setLinearActuatorPosition(ShooterPosition.SHOOT_AMP);
            })
        );
        blank8().onTrue(
            new InstantCommand(() -> {
                System.out.println("blank8 Pressed");
                shooterSubsystem.setLinearActuatorPosition(ShooterPosition.HEIGHT_CHAIN);
                
            })
        );

        shootButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("Shoot Button Pressed");
                shooterSubsystem.operatorEvent(OperatorEvent.FIRE_SPEAKER);
            })
        );

        shootButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("Shoot Speaker Button Realeased");
                shooterSubsystem.operatorEvent(OperatorEvent.NONE);
            })
        );

        prepAmpButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("Prep Amp Button Pressed");
                shooterSubsystem.operatorEvent(OperatorEvent.PREP_AMP);
            })
        );

        prepAmpButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("Prep Amp Button Realeased");
                shooterSubsystem.operatorEvent(OperatorEvent.NONE);
            })
        );

        prepSpeakerSideButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("Shoot Amp Button Pressed");
                shooterSubsystem.operatorEvent(OperatorEvent.PREP_SPEAKER_SIDE);
            })
        );

        prepSpeakerSideButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("Shoot Amp Button Released");
                shooterSubsystem.operatorEvent(OperatorEvent.NONE);
            })
        );

        spitButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("Spit Button Pressed");
                shooterSubsystem.operatorEvent(OperatorEvent.SPIT);
            })
        );
        spitButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("Spit Button Released!");
                shooterSubsystem.operatorEvent(OperatorEvent.NONE);
            })
        );
    }
}