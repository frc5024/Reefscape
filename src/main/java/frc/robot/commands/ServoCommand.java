package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ServoCommand extends Command {

    private final Coral coralSubsystem;

    Servo rampServo = new Servo(Constants.coralConstants.servoChannel);

    public ServoCommand(Coral coralSubsystem) {
        
        this.coralSubsystem = coralSubsystem; 

        addRequirements(coralSubsystem);
    }

    public void initialize() {
        rampServo.set(0);
    }

    public void set90() {
        rampServo.set(0.5);
    }
}
