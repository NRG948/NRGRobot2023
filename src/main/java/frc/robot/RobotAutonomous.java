package frc.robot;

import java.util.EnumSet;

import com.nrg948.autonomous.Autonomous;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Autos;
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
  private final SendableChooser<Integer> scoreCount = new SendableChooser<>();

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

    for (int i = 0; i <= Autos.MAX_GAME_PIECES_TO_SCORE; i++) {
      scoreCount.addOption(String.valueOf(i), 0);
    }

    int numberOfGamePieces = Autos.getNumberOfGamePieces();

    scoreCount.setDefaultOption(String.valueOf(numberOfGamePieces), numberOfGamePieces);

  }

  /**
   * Returns the autonomous command selected in the chooser.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Use ProxyCommand to run the autonomous routine. This allow multiple runs of
    // the same command with potentially different initial delays. It is mainly
    // useful for debugging.
    Command autoCommand = autonomousCommandChooser.getSelected();
    System.out.println(
      "AUTO ROUTINE: " + autoCommand.getName() +
      ", NO. GAME PIECES: " + Autos.getNumberOfGamePieces() +
      ", BALANCE: " + Autos.getBalanceOnChargingStation());
    return getSelectedDelayCommand().andThen(new ProxyCommand(autoCommand));
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
   * Return the number of game pieces the robot should score during autonomous.
   * 
   * @return The number of game pieces the robot should score during autonomous.
   */
  public int getScoreCount() {
    return scoreCount.getSelected();
  }

  /**
   * Adds the autonomous layout to the Shuffleboard tab.
   * 
   * @param layout The layout to add user interface elements.
   */
  public ShuffleboardLayout addShuffleboardLayout(ShuffleboardTab tab) {
    // Add the Autonomous layout to the specified tab.
    ShuffleboardLayout autonomousLayout = tab.getLayout("Autonomous", BuiltInLayouts.kList);

    autonomousLayout.add("Routine", autonomousCommandChooser);
    autonomousLayout.add("Delay", autoDelayChooser);

    ComplexWidget numberOfGamePiecesWidget = autonomousLayout.add("Number of Game Pieces", scoreCount);
    SimpleWidget balanceWidget = autonomousLayout
        .add("Balance on Charging Station", Autos.getBalanceOnChargingStation())
        .withWidget(BuiltInWidgets.kToggleSwitch);

    // Set up a listener to update the number of game pieces to score depending on
    // the selected item in the chooser.
    NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    NetworkTableEntry numberOfGamePiecesEntry = ntInstance
        .getTable(Shuffleboard.kBaseTableName)
        .getSubTable(tab.getTitle())
        .getSubTable(autonomousLayout.getTitle())
        .getSubTable(numberOfGamePiecesWidget.getTitle())
        .getEntry("active");

    ntInstance.addListener(
        numberOfGamePiecesEntry,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll, NetworkTableEvent.Kind.kImmediate),
        (event) -> Autos.setNumberOfGamePieces(Integer.parseInt(event.valueData.value.getString())));

    // Set up a listener to update whether to balance on the charging station
    // depending on the toggle switch.
    GenericEntry balanceEntry = balanceWidget.getEntry();
    BooleanTopic balanceTopic = new BooleanTopic(balanceEntry.getTopic());
    ntInstance.addListener(
        balanceTopic,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll, NetworkTableEvent.Kind.kImmediate),
        (event) -> Autos.setBalanceOnChargingStation(event.valueData.value.getBoolean()));

    return autonomousLayout;
  }
}
