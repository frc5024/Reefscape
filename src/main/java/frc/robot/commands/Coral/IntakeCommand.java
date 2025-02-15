
package frc.robot.commands.Coral;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.ILEDPreset;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import frc.robot.Constants.LEDsConstants;
import frc.lib.leds.LEDPreset;
import frc.robot.commands.LEDs.flashLEDS;
import frc.robot.commands.LEDs.setLEDS;
import frc.robot.commands.LEDs.setLEDSDefault;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Coral;


public class IntakeCommand extends Command { 

    private final Coral coralSubsystem;
    private LEDs leds;
    private ILEDPreset colour;
    private final LEDs s_LEDs = LEDs.getInstance();
    //private static DigitalInput linebreakBottom;
    
    //constructor for IntakeCommand
    public IntakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        addRequirements(coralSubsystem);
    }
    
    //initialize, when command starts, if line is not broken, set state to IDLE
    @Override
    public void initialize() {
        if(!coralSubsystem.isLineBroken()) {
            coralSubsystem.set(Constants.coralConstants.intakeSpeed);
        }else {
            cancel();
        }

    }
    //execute, if button is pressed, startIntake()
    @Override
    public void execute() {
        
    } 


    
    //end, when command ends, set Idle
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();
        //when command ends, set the leds to red
        s_LEDs.setCommand(LEDPreset.Solid.kRed);
        
    }

    //if line is broken, return true, else return false
    @Override
    public boolean isFinished() {
        if(coralSubsystem.isLineBroken()){
            return true;
        }
        return false;
    }
    
}