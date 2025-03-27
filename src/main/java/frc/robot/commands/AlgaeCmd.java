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

    public AlgaeCmd(Algae algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;

        speed = Constants.AlgaeConstant.algaeSpeed;
    }

    public void initialize() {
        timer.restart();
        toggleExtending = !toggleExtending;

        if (toggleExtending) {
            algaeSubsystem.start(speed);
        } else {
            algaeSubsystem.start(-speed);
        }
    }

    public void execute() {
    }

    public void end() {
        algaeSubsystem.stop();
        timer.reset();
    }

    public boolean isFinished() {
        if (timer.hasElapsed(3)) {
            return true;
        } else {
            if (toggleExtending) {
                return algaeSubsystem.isExtended();
            } else {
                return algaeSubsystem.isRetracted();
            }
        }
    }
}