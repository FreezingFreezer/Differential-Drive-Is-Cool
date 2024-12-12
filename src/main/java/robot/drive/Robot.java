package robot.drive;


import static edu.wpi.first.units.Units.Seconds;
import static robot.Constants.PERIOD;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lib.CommandRobot;
import lib.FaultLogger;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;

import com.revrobotics.RelativeEncoder;

import robot.Ports.OI;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Logged {
  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);
  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
  private final PowerDistribution pdh = new PowerDistribution();
    @Log.NT 
  private final Field2d field2d = new Field2d();

  // SUBSYSTEMS
  Drive drive = new Drive();
  private void configureBindings() {
    drive.setDefaultCommand(drive.drive(driver::getLeftY, driver::getRightY));
  }
  // COMMANDS

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(PERIOD.in(Seconds));
    configureGameBehavior();
    configureBindings();
  }

  /** Configures basic behavior for different periods during the game. */
  private void configureGameBehavior() {
    // TODO: Add configs for all additional libraries, components, intersubsystem interaction
    // Configure logging with DataLogManager, Monologue, URCL, and FaultLogger
    DataLogManager.start();
    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, PERIOD.in(Seconds));
    addPeriodic(FaultLogger::update, 2);

    SmartDashboard.putData(CommandScheduler.getInstance());
    // Log PDH
    SmartDashboard.putData("PDH", pdh);
    FaultLogger.register(pdh);

    RobotController.setBrownoutVoltage(6.0);

    if (isReal()) {
      URCL.start();
      pdh.clearStickyFaults();
      pdh.setSwitchableChannel(true);
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /** Configures trigger -> command bindings. */

  /**
   * Command factory to make both controllers rumble.
   *
   * @param rumbleType The area of the controller to rumble.
   * @param strength The intensity of the rumble.
   * @return The command to rumble both controllers.
   */
  public Command rumble(RumbleType rumbleType, double strength) {
    return Commands.runOnce(
            () -> {
              driver.getHID().setRumble(rumbleType, strength);
              operator.getHID().setRumble(rumbleType, strength);
            })
        .andThen(Commands.waitSeconds(0.3))
        .finallyDo(
            () -> {
              driver.getHID().setRumble(rumbleType, 0);
              operator.getHID().setRumble(rumbleType, 0);
            });
  }
  

  @Override
  public void close() {
    super.close();
  }
  public void drive(double leftSpeed, double rightSpeed, SimpleMotorFeedforward feedforward, PIDController leftPIDController, PIDController rightPIDController) {
    final double realLeftSpeed = leftSpeed * DriveConstants.MAX_SPEED;
    final double realRightSpeed = rightSpeed * DriveConstants.MAX_SPEED;
    
      final double leftFeedforward = feedforward.calculate(realLeftSpeed);
      final double rightFeedforward = feedforward.calculate(realRightSpeed);
  
      final double leftPID = 
        leftPIDController.calculate(leftEncoder.getVelocity(), realLeftSpeed);
      final double rightPID = 
        rightPIDController.calculate(rightEncoder.getVelocity(), realRightSpeed);
    }
    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, kDefaultPeriod);
    addPeriodic(FaultLogger::update, 1);
    
}
