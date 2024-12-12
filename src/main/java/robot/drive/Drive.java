package robot.drive;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.Ports;
import com.revrobotics.CANSparkBase.IdleMode;

public class Drive extends SubsystemBase{
    private final AnalogGyro gyro = new AnalogGyro(Ports.Drive.GYRO_CHANNEL);
    private final DifferentialDriveOdometry odometry;
    private final CANSparkMax leftLeader = new CANSparkMax(Ports.Drive.LEFT_LEADER, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(Ports.Drive.LEFT_FOLLOWER, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(Ports.Drive.RIGHT_LEADER, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(Ports.Drive.RIGHT_FOLLOWER, MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
      private final DifferentialDrivetrainSim driveSim;

    
    
    public Drive(){
        gyro.reset();
            odometry = new DifferentialDriveOdometry(
            new Rotation2d(), 
            0, 
            0, 
            new Pose2d());
            
        for (CANSparkMax spark : List.of(leftLeader, leftFollower, rightLeader, rightFollower)){
            spark.restoreFactoryDefaults();
            spark.setIdleMode(IdleMode.kBrake);
        }
        rightFollower.follow(rightLeader);
        leftFollower.follow(leftLeader);

            leftLeader.setInverted(true);
            leftEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);
            rightEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);
        
            leftEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
            rightEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
            leftEncoder.setPosition(0);
            rightEncoder.setPosition(0);
    }

    private void drive(double leftSpeed, double rightSpeed, SimpleMotorFeedforward feedforward, PIDController leftPIDController, PIDController rightPIDController){
        leftLeader.set(leftSpeed);
        rightLeader.set(rightSpeed);
        final double realLeftSpeed = leftSpeed * DriveConstants.MAX_SPEED;
	    final double realRightSpeed = rightSpeed * DriveConstants.MAX_SPEED;
	
        final double leftFeedforward = feedforward.calculate(realLeftSpeed);
        final double rightFeedforward = feedforward.calculate(realRightSpeed);

        final double leftPID = leftPIDController.calculate(leftEncoder.getVelocity(), realLeftSpeed);
        final double rightPID = rightPIDController.calculate(rightEncoder.getVelocity(), realRightSpeed);
        double leftVoltage = leftPID + leftFeedforward;
        double rightVoltage = rightPID + rightFeedforward;
  
        leftLeader.setVoltage(leftVoltage);
        rightLeader.setVoltage(rightVoltage);
        driveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getMiniCIM(2),
            DriveConstants.GEARING,
            DriveConstants.MOI,
            DriveConstants.DRIVE_MASS,
            DriveConstants.WHEEL_RADIUS,
            DriveConstants.TRACK_WIDTH,
            DriveConstants.STD_DEVS);
            driveSim.setInputs(leftVoltage, rightVoltage);
    }
    

    public Command drive(DoubleSupplier vLeft, DoubleSupplier vRight){
        return run(() -> drive(vLeft.getAsDouble(), vRight.getAsDouble()));
    }
    private void updateOdometry(Rotation2d rotation) {
        odometry.update(rotation, leftEncoder.getPosition(), rightEncoder.getPosition());
      }
      @Override 
      public void periodic() {
        updateOdometry(gyro.getRotation2d());
      }
      public Pose2d pose() {
        return odometry.getPoseMeters();
      }
      public static final class FF {
        public static final double kS = 1;
        public static final double kV = 3;
      }
      public static final class PID {
        public static final double kP = 8.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      } 
      @Override
      public void simulationPeriodic() {
        // sim.update() tells the simulation how much time has passed
        driveSim.update(Constants.PERIOD.in(Seconds));
        leftEncoder.setPosition(driveSim.getLeftPositionMeters());
        rightEncoder.setPosition(driveSim.getRightPositionMeters());
      }
}

