package robot.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;
import robot.drive.Drive.FF;
import robot.drive.Drive.PID;

public class DriveConstants {
  public static final double WHEEL_RADIUS = 0.08; //Meters
  public static final double CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS;
  public static final double GEARING = 8.0;
  public static final double POSITION_FACTOR = CIRCUMFERENCE * GEARING;
  public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.0;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FF.kS, FF.kV);
  private final PIDController leftPIDController = new PIDController(PID.kP, PID.kI, PID.kD);
  private final PIDController rightPIDController = new PIDController(PID.kP, PID.kI, PID.kD);
  public static final double MAX_SPEED = 2; // Meters per second
  public static final double TRACK_WIDTH = 0.7112; // Meters
  public static final double MOI = 7.5;
  public static final double DRIVE_MASS = 60.0; //kg
  public static final Matrix<N7, N1> STD_DEVS = VecBuilder.fill(0, 0, 0, 0, 0, 0, 0);
}
