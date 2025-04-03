package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Constants.DriveConstants.DriveType;
import frc.robot.Constants.DriveConstants.WheelIndex;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.units.Units;

public class DriveSubsystem extends SubsystemBase {

    private InputSubsystem input;
    private double maxDriveSpeed;
    private double maxTurnSpeed;
    private DriveType driveType;
    private DifferentialDrive differentialDrive;
    private DifferentialDrive followDifferentialDrive;

    /**
     * <p> The list of PWM Spark Max motor controllers controlling the differential
     * drive motors. </p>
     *
     * <p> This array will be null if we are using a swerve drive. </p>
     */
    private List<SparkMax> differentialDriveMotors;

    /**
     * The list of Spark Maxes controlling the swerve drive motors.
     */
    private List<SparkMax> swerveDriveMotors;

    /**
     * The list of Spark Maxes controlling the swerve pivot motors.
     *
     * <p> This list will be null if we are using a differential drive. </p>
     */
    private List<SparkMax> swervePivotMotors;

    /**
     * To read each swerve module's pivot angle, we will use CANCoders (we will
     * ignore the built-in {@link RelativeEncoder}) because the CANCoders are
     * absolute encoders. This will ensure that each swerve module is reset to
     * the same position each time.
     *
     * <p> This list will be null if we are using a differential drive. </p>
     */
    private List<CANcoder> swerveCANCODER;

    private List<SwerveModuleState> swerveStates;

    /**
     * Our CANCoders measure absolute angles that don't change even when the
     * robot is power-cycled. However, the CANCoder's zero position is not
     * necessarily the same as the actual swerve module zero position. I mean,
     * maybe it is, maybe it isn't!
     *
     * Where we notice a discrepancy, we will record the actual angle the pivot
     * motor needs in order for the swerve module to face directly forward
     * (angle = zero degrees.)
     */
    private double[] CAN_CODER_ANGLE_OFFSETS = { // TODO: These numbers are made-up.
        -61,  // FRONT_LEFT
        -59,  // FRONT_RIGHT
        0,    // BACK_RIGHT
        12    // BACK_LEFT
    };

    /**
     * PID controllers. We'll use four PID Controllers with the same constants
     * for each pivot motor. We'll use the CANCoder's absolute angle as the measurement.
     */
    private List<PIDController> pivotMotorPIDControllers;

    /**
     * Determines whether or not the shuffle board values should affect the wheels
     * of the robot based on if the joystick is being moved.
     */
    private boolean canShuffleBoardActuate;

    private double clampedXAxis;
    private double clampedYAxis;
    private double clampedTurn;
    private double currentYAxis;
    private double currentTurn;

    private List<CANcoderSimState> simEncoders;

    /**
     * Intended to be owned by the RobotContainer and to be used by
     * RobotContainer's commands
     * @param inputSubsystem Provides human input for teleop regardless of the
     * drive type being used.
     * @param driveType the type of drive being used (swerve or differential)
     */
    public DriveSubsystem(InputSubsystem inputSubsystem, DriveType driveType) {
        super("swerveDrive");
        this.driveType = driveType;
        input = inputSubsystem;
        canShuffleBoardActuate = false;
        switch (driveType) {
            case DIFFERENTIAL_DRIVE: {
                final int FRONT_LEFT_ID = DriveConstants.DRIVE_MOTOR_CAN_OFFSET + DriveConstants.WheelIndex.FRONT_LEFT.label;
                final int FRONT_RIGHT_ID = DriveConstants.DRIVE_MOTOR_CAN_OFFSET + DriveConstants.WheelIndex.FRONT_RIGHT.label;
                final int BACK_RIGHT_ID = DriveConstants.DRIVE_MOTOR_CAN_OFFSET + DriveConstants.WheelIndex.BACK_RIGHT.label;
                final int BACK_LEFT_ID = DriveConstants.DRIVE_MOTOR_CAN_OFFSET + DriveConstants.WheelIndex.BACK_LEFT.label;
                differentialDriveMotors = Arrays.asList(new SparkMax[] {
                    new SparkMax(FRONT_RIGHT_ID, MotorType.kBrushed),
                    new SparkMax(BACK_RIGHT_ID, MotorType.kBrushed),
                    new SparkMax(BACK_LEFT_ID, MotorType.kBrushed),
                    new SparkMax(FRONT_LEFT_ID, MotorType.kBrushed)
                });

                // We are using two different Configs for the left and right
                // motors as sometimes they will be doing different things at
                // times. For example, when turning, the left and the right side
                // will be moving in the opposite directions.
                SparkMaxConfig commonConfig = new SparkMaxConfig();
                commonConfig.idleMode(IdleMode.kBrake);
                SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
                SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
                
                SparkMaxConfig followConfig = new SparkMaxConfig();
                followConfig.idleMode(IdleMode.kBrake);
                followConfig.inverted(true);

                leftFollowerConfig.follow(FRONT_LEFT_ID, true).apply(commonConfig);
                rightFollowerConfig.follow(FRONT_RIGHT_ID).apply(commonConfig);
                differentialDriveMotors.get(2).configure(commonConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                differentialDriveMotors.get(3).configure(commonConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                differentialDriveMotors.get(0).configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                differentialDriveMotors.get(1).configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                // differentialDriveMotors.get(DriveConstants.WheelIndex.FRONT_LEFT.label).configure(commonConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                // differentialDriveMotors.get(DriveConstants.WheelIndex.BACK_LEFT.label).configure(commonConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                // differentialDriveMotors.get(DriveConstants.WheelIndex.FRONT_RIGHT.label).configure(commonConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                // differentialDriveMotors.get(DriveConstants.WheelIndex.BACK_RIGHT.label).configure(commonConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                
                differentialDrive = new DifferentialDrive(differentialDriveMotors.get(2),
                                                          differentialDriveMotors.get(0));
                followDifferentialDrive = new DifferentialDrive(differentialDriveMotors.get(3),
                                                                differentialDriveMotors.get(1));
                break;
            }
            case SWERVE_DRIVE: {
                final int FRONT_LEFT = DriveConstants.WheelIndex.FRONT_LEFT.label;
                final int FRONT_RIGHT = DriveConstants.WheelIndex.FRONT_RIGHT.label;
                final int BACK_RIGHT = DriveConstants.WheelIndex.BACK_RIGHT.label;
                final int BACK_LEFT = DriveConstants.WheelIndex.BACK_LEFT.label;

                // Initialize drive motors
                swerveDriveMotors = Arrays.asList(new SparkMax[] {
                    new SparkMax(Constants.DriveConstants.DRIVE_MOTOR_CAN_OFFSET + FRONT_LEFT, MotorType.kBrushless),
                    new SparkMax(Constants.DriveConstants.DRIVE_MOTOR_CAN_OFFSET + FRONT_RIGHT, MotorType.kBrushless),
                    new SparkMax(Constants.DriveConstants.DRIVE_MOTOR_CAN_OFFSET + BACK_RIGHT, MotorType.kBrushless),
                    new SparkMax(Constants.DriveConstants.DRIVE_MOTOR_CAN_OFFSET + BACK_LEFT, MotorType.kBrushless)
                });

                // Initialize pivot motors
                swervePivotMotors = Arrays.asList(new SparkMax[] {
                    new SparkMax(Constants.DriveConstants.PIVOT_MOTOR_CAN_OFFSET + FRONT_LEFT, MotorType.kBrushless),
                    new SparkMax(Constants.DriveConstants.PIVOT_MOTOR_CAN_OFFSET + FRONT_RIGHT, MotorType.kBrushless),
                    new SparkMax(Constants.DriveConstants.PIVOT_MOTOR_CAN_OFFSET + BACK_RIGHT, MotorType.kBrushless),
                    new SparkMax(Constants.DriveConstants.PIVOT_MOTOR_CAN_OFFSET + BACK_LEFT, MotorType.kBrushless)
                });

                // Initialize the pivot motors in a similar manner to how we
                // initialized them for the 2020bot.
                SparkMaxConfig config = new SparkMaxConfig();
                config.idleMode(IdleMode.kBrake);
                //TODO: This belongs in drive motor configuration for getting
                //the drive speed in meters per second.
                //config.encoder.velocityConversionFactor(getConversionFactor());
                swervePivotMotors.forEach(motor -> {
                    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                });

                // Initialize CANcoders
                swerveCANCODER = Arrays.asList(new CANcoder[] {
                    new CANcoder(Constants.DriveConstants.PIVOT_MOTOR_CAN_CODER_CAN_ID_OFFSET + FRONT_LEFT),
                    new CANcoder(Constants.DriveConstants.PIVOT_MOTOR_CAN_CODER_CAN_ID_OFFSET + FRONT_RIGHT),
                    new CANcoder(Constants.DriveConstants.PIVOT_MOTOR_CAN_CODER_CAN_ID_OFFSET + BACK_RIGHT),
                    new CANcoder(Constants.DriveConstants.PIVOT_MOTOR_CAN_CODER_CAN_ID_OFFSET + BACK_LEFT)
                });

                pivotMotorPIDControllers = Arrays.asList(new PIDController[] {
                    new PIDController(Constants.DriveConstants.PIVOT_MOTOR_P, Constants.DriveConstants.PIVOT_MOTOR_I, Constants.DriveConstants.PIVOT_MOTOR_D),
                    new PIDController(Constants.DriveConstants.PIVOT_MOTOR_P, Constants.DriveConstants.PIVOT_MOTOR_I, Constants.DriveConstants.PIVOT_MOTOR_D),
                    new PIDController(Constants.DriveConstants.PIVOT_MOTOR_P, Constants.DriveConstants.PIVOT_MOTOR_I, Constants.DriveConstants.PIVOT_MOTOR_D),
                    new PIDController(Constants.DriveConstants.PIVOT_MOTOR_P, Constants.DriveConstants.PIVOT_MOTOR_I, Constants.DriveConstants.PIVOT_MOTOR_D)
                });

                if (RobotBase.isSimulation()) {
                    simEncoders = new ArrayList<CANcoderSimState>();
                    for(CANcoder encoder : swerveCANCODER) {
                        simEncoders.add(new CANcoderSimState(encoder));
                    }
                }
                break;
            }
        }
        SmartDashboard.putData(this);
    }

    private static double getConversionFactor() {
         // - A conversion factor of 1.0 will bypass conversion (i.e.,
        //   getVelocity() would return values in units of RPM.)
        //
        //   So how do we convert revolutions per minute to meters per second?
        //
        //   X revolutions    1 minute      (2 * PI * radius) meters        (2 * PI * radius)
        //   ------------- * ----------- * -------------------------- = X * ----------------- meters per second
        //      1 minute      60 seconds          1 revolution                      60

        return (2 * Math.PI * Constants.DriveConstants.SWERVE_MODULE_WHEEL_RADIUS_METERS) / 60;
    }

    @FunctionalInterface
    public interface TriConsumer<T1,T2,T3> {
        public void accept(T1 t1, T2 t2, T3 t3);
    }

    /**
     * Determines the set of values we'll send to and from the shuffleboard.
     * @param builder The object that will be used to send values to the shuffleboard.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        System.out.printf("initalizing shuffleboard");
        TriConsumer<List<SparkMax>, String, WheelIndex> addMotorHelper = (motors, name, index) -> {
            builder.addDoubleProperty(name,
                                      () -> motors.get(index.label).get(),
                                      (speed) -> {
                                          motors.get(index.label).set(speed);
                                          canShuffleBoardActuate = true;
                                      });
        };
        builder.setSmartDashboardType(this.driveType.toString() == "DIFFERENTIAL_DRIVE" ? "DifferentialDrive" : "SwerveDrive");
        switch (driveType) {
            case DIFFERENTIAL_DRIVE:

                // Add the differential drive motors to the shuffleboard.
                addMotorHelper.accept(differentialDriveMotors, "FR Motor", WheelIndex.FRONT_RIGHT);
                addMotorHelper.accept(differentialDriveMotors, "FL Motor", WheelIndex.FRONT_LEFT);
                addMotorHelper.accept(differentialDriveMotors, "BR Motor", WheelIndex.BACK_RIGHT);
                addMotorHelper.accept(differentialDriveMotors, "BL Motor", WheelIndex.BACK_LEFT);
                break;
            case SWERVE_DRIVE:

                // Add the swerve drive motors to the shuffleboard.
                addMotorHelper.accept(swerveDriveMotors, "FR Drive", WheelIndex.FRONT_RIGHT);
                addMotorHelper.accept(swerveDriveMotors, "FL Drive", WheelIndex.FRONT_LEFT);
                addMotorHelper.accept(swerveDriveMotors, "BR Drive", WheelIndex.BACK_RIGHT);
                addMotorHelper.accept(swerveDriveMotors, "BL Drive", WheelIndex.BACK_LEFT);

                // Add the swerve pivot motors to the shuffleboard.
                addMotorHelper.accept(swervePivotMotors, "FR Pivot", WheelIndex.FRONT_RIGHT);
                addMotorHelper.accept(swervePivotMotors, "FL Pivot", WheelIndex.FRONT_LEFT);
                addMotorHelper.accept(swervePivotMotors, "BR Pivot", WheelIndex.BACK_RIGHT);
                addMotorHelper.accept(swervePivotMotors, "BL Pivot", WheelIndex.BACK_LEFT);

                builder.addStringArrayProperty("Swerve States", () -> {
                    List<String> states = new ArrayList<String>();
                    if (swerveStates != null) {
                        for (SwerveModuleState state : swerveStates) {
                            states.add(state.toString());
                        }
                    }
                    return states.toArray(new String[0]);
                }, (value) -> {});
                break;
        }
        builder.setActuator(true);
        builder.setSafeState(this::disable);
    }

    /**
     * Calling this function removes the shuffleboard's ability to write values
     * to the motors. This is called by our sendable interface so that the only
     * time a shuffleboard live window can set motor speeds, is during test
     * mode.
     */
    void disable() {
        switch (driveType) {
            case DIFFERENTIAL_DRIVE:
                differentialDriveMotors.forEach(SparkMax::disable);
                break;
            case SWERVE_DRIVE:
                swerveDriveMotors.forEach(SparkMax::disable);
                swervePivotMotors.forEach(SparkMax::disable);
                break;
        }
    }

    /**
     * Drive the robot according to the given speeds.
     *
     * @param xAxis The speed at which the robot should move left or right. 1.0
     * should be full speed to the right and -1.0 should be full speed to the
     * left.
     * @param yAxis The speed at which the robot should move forward or
     * backward. A value of 1.0 means full speed forward and -1.0 means full
     * speed backward.
     * @param turn The speed at which the robot should turn (1.0 is full speed
     * clockwise and -1.0 is full speed counterclockwise.
     */
    public void drive(double xAxis, double yAxis, double turn) {
        clampedXAxis = MathUtil.clamp(xAxis, -1.0, 1.0);
        clampedYAxis = MathUtil.clamp(yAxis, -0.8, 0.8);
        clampedTurn = MathUtil.clamp(turn, -0.5, 0.5);
    }

    /**
     * Update the position of the drive according to human input (during teleop)
     * or according to the current trajectory (during autonomous).
     */
    public void periodic() {
        switch (driveType) {
            case DIFFERENTIAL_DRIVE:
                // If the joystick is being moved, then the shuffleboard will be
                // prevented from setting anything. This is to prevent the
                // problem of the arcadeDrive() overriding the values that the
                // shuffleboard sets.
                if (DriverStation.isTeleopEnabled()) {
                    if (input.getForwardBack() != 0 || input.getTurn() != 0) {
                        System.out.println(input.getForwardBack());
                        this.drive(input.getLeftRight(), input.getForwardBack(), input.getTurn());
                        canShuffleBoardActuate = false;
                    } else if (!canShuffleBoardActuate) {
                        this.drive(0, 0, 0);
                    } else {
                        // canShuffleBoardActuate is true and the driver is not
                        // touching the controls.  Therefore, do _nothing_; this
                        // will permit motor values that were set in the
                        // shuffleboard to 'escape' into the actual robot without
                        // being overwritten.
                    }
                } else {
                    // control makes it here if we're either in autonomous or
                    // testing with shuffleboard.
                }

                // We have this if statement for a reason. It is here because if teleop isn't
                // enabled (autonomous or test), then the code will automatically exit
                // everything above. If we didn't have the code below inside the if statement,
                // then arcadeDrive will be called regardless of what state the robot is in, and
                // would override any values inputted into the shuffleboard.

                if (!DriverStation.isTestEnabled()) {
                    if (Math.abs(clampedYAxis) < Constants.MathConstants.EPSILON &&
                        Math.abs(clampedTurn) < Constants.MathConstants.EPSILON) {
                        differentialDrive.arcadeDrive(0.0, 0.0);
                        followDifferentialDrive.arcadeDrive(0.0, 0.0);
                        //System.out.println("stopped.");
                    } else {
                        // Limit the amount that the yAxis and turn values are changed
                        // by calculating the difference between the target value, clampedYAxis
                        // and the current value, currentYAxis. Then clamping the difference to
                        // a threshold and adding the clamped difference to the current value.
                        double diffYAxis = clampedYAxis - currentYAxis;
                        diffYAxis = MathUtil.clamp(diffYAxis, -0.05, 0.05);
                        currentYAxis += diffYAxis;
                        
                        double diffTurn = clampedTurn - currentTurn;
                        diffTurn = MathUtil.clamp(diffTurn, -0.25, 0.25);
                        currentTurn += diffTurn;

                        differentialDrive.arcadeDrive(currentYAxis, currentTurn);
                        followDifferentialDrive.arcadeDrive(currentYAxis, currentTurn);
                        System.out.println("y-axis: " + clampedYAxis + " turn: " + clampedTurn);
                    }
                }
                //differentialDriveMotors.get(2).set(0.2);
                //differentialDriveMotors.get(3).set(0.2);
                
                // differentialDriveMotors.get(0).set(0.2);
                // differentialDriveMotors.get(1).set(0.2);
                
                break;
            case SWERVE_DRIVE:
                // Convert the human input into a ChassisSpeeds object giving us
                // the overall bearing of the chassis.
                ChassisSpeeds movement =
                    new ChassisSpeeds(Constants.DriveConstants.SWERVE_DRIVE_MAX_DRIVING_SPEED * input.getForwardBack(),
                                      Constants.DriveConstants.SWERVE_DRIVE_MAX_DRIVING_SPEED * input.getLeftRight(),
                                      Constants.DriveConstants.SWERVE_DRIVE_MAX_TURNING_SPEED * input.getTurn());

                // With inverse kinematics, convert the overall chassis speed
                // into the speeds and angles for all four swerve modules.
                final int FRONT_LEFT = DriveConstants.WheelIndex.FRONT_LEFT.label;
                final int FRONT_RIGHT = DriveConstants.WheelIndex.FRONT_RIGHT.label;
                final int BACK_RIGHT = DriveConstants.WheelIndex.BACK_RIGHT.label;
                final int BACK_LEFT = DriveConstants.WheelIndex.BACK_LEFT.label;
                SwerveDriveKinematics kinematics =
                    new SwerveDriveKinematics(Constants.DriveConstants.SWERVE_MODULE_POSITIONS.get(FRONT_LEFT),
                                              Constants.DriveConstants.SWERVE_MODULE_POSITIONS.get(FRONT_RIGHT),
                                              Constants.DriveConstants.SWERVE_MODULE_POSITIONS.get(BACK_RIGHT),
                                              Constants.DriveConstants.SWERVE_MODULE_POSITIONS.get(BACK_LEFT));

                swerveStates = Arrays.asList(kinematics.toSwerveModuleStates(movement));

                double[] CANCoderAngles = new double[4];
                for (int i = 0; i < 4; i++) {
                   var temporary = swerveCANCODER.get(i).getAbsolutePosition();
                   temporary.refresh();
                   CANCoderAngles[i] = temporary.getValueAsDouble();
                   double angleDiff = CANCoderAngles[i] - swerveStates.get(i).angle.getDegrees();
                   if (angleDiff > 1.0) {
                       swervePivotMotors.get(i).set(1.0);
                   } else if (angleDiff < -1.0) {
                       swervePivotMotors.get(i).set(1.0);
                   } else {
                       swervePivotMotors.get(i).set(0.0);
                   }
                }

                // TODO: Use the CANCoder's measurements for the PID
                // controllers. We still don't have the idea of goal angles (our
                // setpoint) yet.
                //
                // Later, we should InItsendable to send our piviot angles to
                // the suffleboard for easier debugging.

                break;
            default:
                break;
        }
    }

    public void simulationPeriodic() {
        for (int i = 0; i < 4; ++i) {
            var simEncoder = simEncoders.get(i);
            var swervePivotMotor = swervePivotMotors.get(i);
            simEncoder.addPosition(Units.Degrees.of(swervePivotMotor.get()));
        }
    }
}
