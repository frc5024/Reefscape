package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Coral.coralState;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;


public class OuttakeCommand extends Command { 

    private final Coral coralSubsystem;
    private static DigitalInput linebreakTop;
    private static DigitalInput linebreakBottom;
    
    //constants for outtakeTime
    int outtakeTime = Constants.outtakeConstants.outtakeTime;

    //constructor for OuttakeCommand
    public OuttakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem; 
        //linebreakTop = new DigitalInput(Constants.coralConstants.linebreakTopChannel);
        //linebreakBottom = new DigitalInput(Constants.coralConstants.linebreakBottomChannel);

        addRequirements(coralSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(IntakeCommand.isTopLineBroken()) {
            coralSubsystem.state = coralState.HOLDING;
        }

        activeOuttake(false);

    }
    //execute, if line is not broken, and t2nd linebreak not broken, set activeOuttake to false and state to IDLE
    @Override
    public void execute() {
        if(!isTopLineBroken()) {
            if(!isBottomLineBroken()) {
                activeOuttake(true);
                coralSubsystem.state = coralState.IDLE;
                coralSubsystem.setIdle();
            }
        }
    }
        
    //end, when command ends, set activeOuttake to false and set state to IDLE
    @Override
    public void end(boolean interrupted) {
        activeOuttake(false);
        coralSubsystem.state = coralState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public static boolean isTopLineBroken() {
        return linebreakTop.get();
    }

    public static boolean isBottomLineBroken() {
        return linebreakBottom.get();
    }
    //method for activeOuttake, 
    public void activeOuttake(boolean outtaking) {
        //if line is broken and outtaking is true, start outtake and set state to DROPPING
        if(isTopLineBroken() && isBottomLineBroken() && outtaking) {
            coralSubsystem.startOuttake(true);
            coralSubsystem.state = coralState.DROPPING;
        }
        //else, stop outtake and set state to IDLE
        else {
            coralSubsystem.startOuttake(false);
            coralSubsystem.state = coralState.IDLE;
        }
    }
}