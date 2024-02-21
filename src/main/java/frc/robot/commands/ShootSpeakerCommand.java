package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeakerCommand extends Command {

    private ShooterSubsystem mShooter;
    public ShootSpeakerCommand(ShooterSubsystem shooterSubsystem) {
        mShooter = shooterSubsystem;
        System.out.println("shootSpeaker()");
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        System.out.println("initialize()");
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        System.out.println("execute()");
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        System.out.println("end()");
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        System.out.println("isFinished()");
        return true;
    }
}
