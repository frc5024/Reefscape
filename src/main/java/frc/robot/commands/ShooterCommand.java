package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
    private final Shooter m_shooter;
    private DoubleSupplier outPower;
    private DoubleSupplier inPower;

    public ShooterCommand(Shooter shooter, DoubleSupplier outPower, DoubleSupplier inPower) {
        m_shooter = shooter;
        this.outPower = outPower;
        this.inPower = inPower;
        addRequirements(m_shooter);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double motor1Speed = (outPower.getAsDouble() / 2) + (-inPower.getAsDouble() / 2);
        m_shooter.setSpeed(motor1Speed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
