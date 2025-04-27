package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Turret;
import frc.robot.RobotContainer;

public class StickRotationCommand extends Command {

    private final Turret turretSubsystem;

    CommandXboxController operator = RobotContainer.operator;
    
    public StickRotationCommand(Turret turretSubsystem) {
        this.turretSubsystem = turretSubsystem;

        addRequirements(turretSubsystem);
    }

    public void initialize() {
        turretSubsystem.setIdle();
    }
    
    public void execute() {
        turretSubsystem.updateJoystick(operator.getHID().getRawAxis(2));

    }

}
