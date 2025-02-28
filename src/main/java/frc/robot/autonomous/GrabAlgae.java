package frc.robot.autonomous;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * 
 */
public class GrabAlgae {
    private final AlgaeSubsystem algaeSubsystem;

    /**
     * 
     */
    public GrabAlgae(AlgaeSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
    }

    /**
     * 
     */
    public Command getAutoCommand() {
        List<PathPlannerPath> pathGroup = null;

        try {

            pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("GrabAlgae");

        } catch (Exception e) {
        }

        if (pathGroup == null) {
            return Commands.print("*** Failed to build GrabAlgae");
        }

        Command command = Commands.sequence(
                Commands.print("*** Starting GrabAlgae ***"),

                AutoBuilder.followPath(pathGroup.get(0)),
                new InstantCommand(() -> {
                    this.algaeSubsystem.addAction(AlgaeSubsystem.Action.INTAKE);
                }),
                new WaitUntilCommand(this.algaeSubsystem::hasAlgae),
                AutoBuilder.followPath(pathGroup.get(1)),
                new InstantCommand(() -> {
                    this.algaeSubsystem.addAction(AlgaeSubsystem.Action.EJECT);
                }),

                Commands.print("*** Finished GrabAlgae ***"));

        return command;
    }
}
