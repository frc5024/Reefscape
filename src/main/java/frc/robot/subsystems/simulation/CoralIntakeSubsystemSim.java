package frc.robot.subsystems.simulation;

import frc.lib.statemachine.StateMetadata;
import frc.robot.modules.coral.CoralIntakeModuleIO;
import frc.robot.modules.elevator.ElevatorVisualizer;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.utils.MapleSimUtil;

public class CoralIntakeSubsystemSim extends CoralIntakeSubsystem {
    /**
     * 
     */
    public CoralIntakeSubsystemSim(CoralIntakeModuleIO intakeModule) {
        super(intakeModule);
    }

    @Override
    protected void handleEject(StateMetadata<Action> stateMetadata) {
        super.handleEject(stateMetadata);

        if (stateMetadata.isFirstRun()) {
            if (MapleSimUtil.getCoralIntakeSimulation().obtainGamePieceFromIntake()) {
                MapleSimUtil.ejectCoral(ElevatorVisualizer.getCoralPose("Measured"));
            }
        }
    }

    @Override
    protected void handleIntake(StateMetadata<Action> stateMetadata) {
        super.handleIntake(stateMetadata);

        if (stateMetadata.isFirstRun()) {
            MapleSimUtil.getCoralIntakeSimulation().startIntake();
        }
    }

    @Override
    protected void handleStop(StateMetadata<Action> stateMetadata) {
        super.handleStop(stateMetadata);

        if (stateMetadata.isFirstRun()) {
            MapleSimUtil.getCoralIntakeSimulation().stopIntake();
        }
    }

    @Override
    public boolean hasCoral() {
        return MapleSimUtil.getCoralIntakeSimulation().getGamePiecesAmount() != 0;
    }

    @Override
    public boolean hasEjected() {
        return !hasCoral();
    }

    /**
     * Used in autonomous simulations
     */
    public void setHasCoral(boolean has_coral) {
        boolean added = false;
        if (has_coral) {
            added = MapleSimUtil.getCoralIntakeSimulation().addGamePieceToIntake();
        }
        super.setHasCoral(has_coral && added);
    }
}
