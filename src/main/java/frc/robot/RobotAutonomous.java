package frc.robot;

import java.util.EnumSet;

import com.nrg948.autonomous.Autonomous;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Subsystems;

/**
 * This class creates and manages the user interface operators use to select
 * and configure autonomous routines.
 */
public class RobotAutonomous {

    /**
     * An enum representing the delay, in seconds, before the autonomous routine
     * starts. This can be used to avoid collisions with alliance robots when we
     * know their autonomous path would cross ours.
     */
    private static enum ChooseAutoDelay {
        NO_DELAY(0),
        DELAY_2_SECONDS(2),
        DELAY_5_SECONDS(5);

        private int delay;

        /**
         * Creates a new ChooseAuthoDelay.
         * 
         * @param delay The delay in seconds.
         */
        ChooseAutoDelay(int delay) {
            this.delay = delay;
        }

        /**
         * Returns the delay in seconds.
         * 
         * @return The delay in seconds.
         */
        public int getDelay() {
            return delay;
        }

        /**
         * Returns the name of this constant.
         */
        @Override
        public String toString() {
            return String.format("%d seconds", delay);
        }
    }

    private final Subsystems subsystems;
    private final SendableChooser<Command> autonomousCommandChooser;
    private final SendableChooser<ChooseAutoDelay> autoDelayChooser = new SendableChooser<>();

    /**
     * Creates a new RobotAutonomous.
     * 
     * @param subsystems The subsystems container.
     */
    public RobotAutonomous(Subsystems subsystems) {
        this.subsystems = subsystems;

        autonomousCommandChooser = Autonomous.getChooser(this.subsystems, "frc.robot");

        for (ChooseAutoDelay delay : EnumSet.allOf(ChooseAutoDelay.class)) {
            autoDelayChooser.addOption(delay.toString(), delay);
        }

        autoDelayChooser.setDefaultOption(ChooseAutoDelay.NO_DELAY.toString(), ChooseAutoDelay.NO_DELAY);
    }

    /**
     * Returns the autonomous command selected in the chooser.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return getSelectedDelayCommand().andThen(autonomousCommandChooser.getSelected());
    }

    /**
     * Returns a {@link WaitCommand} used to wait before starting the autonomous
     * routine.
     * 
     * @return A {@link WaitCommand} that waits for the selected delay.
     */
    private Command getSelectedDelayCommand() {
        ChooseAutoDelay delayChoice = autoDelayChooser.getSelected();
        return new WaitCommand(delayChoice.getDelay());
    }

    /**
     * Adds the autonomous layout to the Shuffleboard tab.
     * 
     * @param layout The layout to add user interface elements.
     */
    public ShuffleboardLayout addShuffleboardLayout(ShuffleboardTab tab) {
        ShuffleboardLayout autonomousLayout = tab.getLayout("Autonomous", BuiltInLayouts.kList);

        autonomousLayout.add("Routine", autonomousCommandChooser);
        autonomousLayout.add("Delay", autoDelayChooser);

        return autonomousLayout;
    }
}
