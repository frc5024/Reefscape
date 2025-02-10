package frc.robot.autonomous;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * 
 */
public class ClearAlgae {
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
    private final CoralIntakeSubsystem coralIntakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    /**
     * 
     */
    public ClearAlgae(AlgaeIntakeSubsystem algaeIntakeSubsystem, CoralIntakeSubsystem coralIntakeSubsystem,
            ElevatorSubsystem elevatorSubsystem) {
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;
        this.coralIntakeSubsystem = coralIntakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    /**
     * 
     */
    public Command getAutoCommand() {
        List<PathPlannerPath> pathGroup = null;

        try {

            pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("ClearAlgae");

        } catch (Exception e) {
        }

        if (pathGroup == null) {
            return Commands.print("*** Failed to build ClearAlgae");
        }

        // If simulation set coral in
        if (Robot.isSimulation()) {
            this.coralIntakeSubsystem.setHasCoral(true);
            this.algaeIntakeSubsystem.setHasAlgae(false);
        }

        Command command = Commands.sequence(
                Commands.print("*** Starting ClearAlgae ***"),

                new ParallelCommandGroup(
                        AutoBuilder.followPath(pathGroup.get(0)),
                        new InstantCommand(() -> {
                            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_CORAL_1);
                        })),
                new InstantCommand(() -> {
                    this.coralIntakeSubsystem.addAction(CoralIntakeSubsystem.Action.EJECT);
                }),
                new WaitUntilCommand(this.coralIntakeSubsystem::hasEjected),
                new ParallelCommandGroup(
                        AutoBuilder.followPath(pathGroup.get(1)),
                        new InstantCommand(() -> {
                            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_ALGAE_2);
                        })),
                new InstantCommand(() -> {
                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.INTAKE);
                }),
                new WaitUntilCommand(this.algaeIntakeSubsystem::hasAlgae),
                new ParallelCommandGroup(
                        AutoBuilder.followPath(pathGroup.get(2)),
                        new InstantCommand(() -> {
                            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_ALGAE_1);
                        }),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new InstantCommand(() -> {
                                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.EJECT);
                                }))),
                new WaitUntilCommand(this.algaeIntakeSubsystem::hasEjected),
                new InstantCommand(() -> {
                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.INTAKE);
                }),
                new WaitUntilCommand(this.algaeIntakeSubsystem::hasAlgae),
                new ParallelCommandGroup(
                        AutoBuilder.followPath(pathGroup.get(3)),
                        new InstantCommand(() -> {
                            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_ALGAE_2);
                        }),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new InstantCommand(() -> {
                                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.EJECT);
                                }))),
                new WaitUntilCommand(this.algaeIntakeSubsystem::hasEjected),
                new InstantCommand(() -> {
                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.INTAKE);
                }),
                new WaitUntilCommand(this.algaeIntakeSubsystem::hasAlgae),
                new ParallelCommandGroup(
                        AutoBuilder.followPath(pathGroup.get(4)),
                        new InstantCommand(() -> {
                            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_ALGAE_1);
                        }),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new InstantCommand(() -> {
                                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.EJECT);
                                }))),
                new WaitUntilCommand(this.algaeIntakeSubsystem::hasEjected),
                new InstantCommand(() -> {
                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.INTAKE);
                }),
                new WaitUntilCommand(this.algaeIntakeSubsystem::hasAlgae),
                new ParallelCommandGroup(
                        AutoBuilder.followPath(pathGroup.get(5)),
                        new InstantCommand(() -> {
                            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_ALGAE_2);
                        }),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new InstantCommand(() -> {
                                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.EJECT);
                                }))),
                new WaitUntilCommand(this.algaeIntakeSubsystem::hasEjected),
                new InstantCommand(() -> {
                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.INTAKE);
                }),
                new WaitUntilCommand(this.algaeIntakeSubsystem::hasAlgae),
                new ParallelCommandGroup(
                        AutoBuilder.followPath(pathGroup.get(6)),
                        new InstantCommand(() -> {
                            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_ALGAE_1);
                        }),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new InstantCommand(() -> {
                                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.EJECT);
                                }))),
                new WaitUntilCommand(this.algaeIntakeSubsystem::hasEjected),
                new InstantCommand(() -> {
                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.INTAKE);
                }),

                Commands.print("*** Finished ClearAlgae ***"));

        return command;
    }
}
