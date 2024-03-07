package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ShooterAngleSubsystem.ShooterPosition;
import frc.robot.subsystems.ShooterSubsystem.OperatorEvent;

public class ButtonBoardSubsystem extends SubsystemBase {
    public static final double FINE_STRAFE_DISTANCE_CM = 3.0;

    private enum ButtonBoardOperationMode {
        Drive,
        Camera
    }

    private final int kJoystickChannel1 = 1;
    private final int kJoystickChannel2 = 2;

    private Joystick m_joystick1;
    private Joystick m_joystick2;

    private ButtonBoardOperationMode m_operationMode;

    private static ButtonBoardSubsystem mInstance = null;

    public static ButtonBoardSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ButtonBoardSubsystem();
        }
        return mInstance;
    }

    private ButtonBoardSubsystem() {
        m_joystick1 = new Joystick(kJoystickChannel1);
        m_joystick2 = new Joystick(kJoystickChannel2);
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

    private JoystickButton raiseBenderButton() {
        return new JoystickButton(m_joystick1, 6);
    }

    private JoystickButton lowerBenderButton() {
        return new JoystickButton(m_joystick1, 5);
    }

    private JoystickButton test1Button() {
        return new JoystickButton(m_joystick1, 7);
    }

    private JoystickButton test2Button() {
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

    public double getJoystickX() {
        return m_joystick2.getX();
    }

    public double getJoystickY() {
        return m_joystick2.getY();
    }

    public void configureButtonBindings() {

        driveCameraSwitch().onTrue(
                new InstantCommand(() -> setOperationMode()));

        driveCameraSwitch().onFalse(
                new InstantCommand(() -> setOperationMode()));

        raiseElevatorButton().onTrue(
                new InstantCommand(() -> {
                    System.out.println("raiseElevatorButton Press");
                    ShooterSubsystem.getInstance().raiseElevator();
                }));

        raiseElevatorButton().onFalse(
                new InstantCommand(() -> {
                    System.out.println("raiseElevatorButton Release");
                    ShooterSubsystem.getInstance().stopElevator();
                }));

        lowerElevatorButton().onTrue(
                new InstantCommand(() -> {
                    System.out.println("lowerElevatorButton Press");
                    ShooterSubsystem.getInstance().lowerElevator();
                }));

        lowerElevatorButton().onFalse(
                new InstantCommand(() -> {
                    System.out.println("lowerElevatorButton Release");
                    ShooterSubsystem.getInstance().stopElevator();
                }));

        raiseClimberButton().onTrue(
                new InstantCommand(() -> {
                    System.out.println("raiseClimberButton Press");
                    ClimberSubsystem.getInstance().raiseMotors();
                }));

        raiseClimberButton().onFalse(
                new InstantCommand(() -> {
                    System.out.println("raiseClimberButton Release");
                    ClimberSubsystem.getInstance().stopMotors();
                }));

        lowerClimberButton().onTrue(
                new InstantCommand(() -> {
                    System.out.println("lowerClimberButton Press");
                    ClimberSubsystem.getInstance().lowerMotors();
                }));

        lowerClimberButton().onFalse(
                new InstantCommand(() -> {
                    ClimberSubsystem.getInstance().stopMotors();
                    System.out.println("lowerClimberButton Release");
                }));

        raiseLinearActuatorButton().onTrue(
                new InstantCommand(() -> {
                    System.out.println("raiseLinearActuatorButton Press");
                    ShooterSubsystem.getInstance().raiseLinearActuator();
                }));

        raiseLinearActuatorButton().onFalse(
                new InstantCommand(() -> {
                    System.out.println("raiseLinearActuatorButton Release");
                    ShooterSubsystem.getInstance().stopLinearActuator();
                }));

        lowerLinearActuatorButton().onTrue(
                new InstantCommand(() -> {
                    System.out.println("lowerLinearActuatorButton Press");
                    ShooterSubsystem.getInstance().lowerLinearActuator();
                }));

        lowerLinearActuatorButton().onFalse(
                new InstantCommand(() -> {
                    System.out.println("lowerLinearActuatorButton Release");
                    ShooterSubsystem.getInstance().stopLinearActuator();
                }));

        spitButton().onTrue(
                new InstantCommand(() -> {
                    System.out.println("Spit Button Pressed");
                    ShooterSubsystem.getInstance().operatorEvent(OperatorEvent.SPIT);
                }));

        spitButton().onFalse(
                new InstantCommand(() -> {
                    System.out.println("Spit Button Released");
                    ShooterSubsystem.getInstance().operatorEvent(OperatorEvent.SPIT_STOP);
                }));

        prepSpeakerFrontButton().onTrue(
                new InstantCommand(() -> {
                    System.out.println("Prep Speaker Front Button Pressed");
                    ShooterSubsystem.getInstance().operatorEvent(OperatorEvent.PREP_SPEAKER_FRONT);
                }));

        test1Button().onTrue(
            new InstantCommand(() -> {
                System.out.println("Test 1 Pressed");
                ShooterSubsystem.getInstance().setLinearActuatorPosition(ShooterPosition.SHOOT_PID);
            })
        );
        test2Button().onTrue(
            new InstantCommand(() -> {
                System.out.println("Test 2 Pressed");
                ShooterSubsystem.getInstance().operatorEvent(OperatorEvent.PREP_SOURCE);
                
            })
        );

        shootButton().onTrue(
                new InstantCommand(() -> {
                    System.out.println("Shoot Button Pressed");
                    ShooterSubsystem.getInstance().operatorEvent(OperatorEvent.FIRE);
                }));

        prepAmpButton().onTrue(
                new InstantCommand(() -> {
                    System.out.println("Prep Amp Button Pressed");
                    ShooterSubsystem.getInstance().operatorEvent(OperatorEvent.PREP_AMP);
                }));

        prepSpeakerSideButton().onTrue(
                new InstantCommand(() -> {
                    System.out.println("Shoot Amp Button Pressed");
                    ShooterSubsystem.getInstance().operatorEvent(OperatorEvent.PREP_SPEAKER_SIDE);
                }));

        prepSpeakerSideButton().onFalse(
                new InstantCommand(() -> {
                    System.out.println("Shoot Amp Button Released");
                    ShooterSubsystem.getInstance().operatorEvent(OperatorEvent.NONE);
                }));

        spitButton().onTrue(
                new InstantCommand(() -> {
                    System.out.println("Spit Button Pressed");
                    ShooterSubsystem.getInstance().operatorEvent(OperatorEvent.SPIT);
                }));

        spitButton().onFalse(
                new InstantCommand(() -> {
                    System.out.println("Spit Button Released!");
                    ShooterSubsystem.getInstance().operatorEvent(OperatorEvent.NONE);
        }));

        raiseBenderButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("Raise Bender Button Pressed");
                ShooterSubsystem.getInstance().raiseBender();
            })
        );

        lowerBenderButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("Lower Bender Button Pressed");
                ShooterSubsystem.getInstance().lowerBender();
            })
        );
    }
}