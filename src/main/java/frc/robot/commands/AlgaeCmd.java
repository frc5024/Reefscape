package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;

public class AlgaeCmd extends Command {
    public Algae algaeSubsystem;
    public boolean toggleExtending;
    public double speed;
    public Timer timer = new Timer();

    public AlgaeCmd(Algae algaeSubsystem, boolean toggleExtending) {
        this.algaeSubsystem = algaeSubsystem;
        this.toggleExtending = toggleExtending;

        speed = Constants.AlgaeConstant.algaeSpeed;
    }

    public void initialize() {
        timer.restart();

        if (toggleExtending) {
            algaeSubsystem.start(speed);
        } else {
            algaeSubsystem.start(-speed);
        }
    }

    public void execute() {
    }

    public void end(boolean interrupted) {
        algaeSubsystem.stop();
        timer.reset();
    }

    public boolean isFinished() {
        if (timer.hasElapsed(1)) {
            return true;
        } else {
            // if (toggleExtending) {
            // return algaeSubsystem.isExtended();
            // } else {
            // return algaeSubsystem.isRetracted();
            // }
            return false;
        }
    }
}