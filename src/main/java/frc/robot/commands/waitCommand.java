package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class waitCommand extends Command {

    double setTime = 0;

    public Timer timer = new Timer();
    public boolean done = false;

    public waitCommand(double setTime) {
        this.setTime = setTime;
    }

    @Override
    public void initialize() {
        done = false;
        timer.restart();
    }

    @Override
    public void execute() {
        if (timer.get() >= setTime)
            done = true;
    }

    @Override
    public void end(boolean interrupted) {
        done = false;
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
