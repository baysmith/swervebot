package frc.robot.subsystems;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Constants.DriveConstants.DriveType;
import frc.robot.Constants.DriveConstants.WheelIndex;

public class DriveSubsystem extends SubsystemBase {

    private InputSubsystem input;
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
    private double[] CAN_CODER_ANGLE_OFFSETS = { // These values are all in degrees.
        18.37,  // BACK_RIGHT
        70.40,  // BACK_LEFT
        66.75,  // FRONT_LEFT
        114.35, // FRONT_RIGHT
    };

    /**
     * Allows us to ensure all motors rotate in the correct direction when commanded
     * (FL is inverted as it was rotating opposite the direction assigned to it)
     */
    private boolean[] isMotorReversed = { 
        true, // BACK_RIGHT
        true, // BACK_LEFT
        true,  // FRONT_LEFT (as of 12/14/25)
        true, // FRONT_RIGHT
    };

    /**
     * PID controllers. We'll use four PID Controllers with the same constants
     * for each pivot motor. 
     *
     * We'll use the CANCoder's absolute angle as the measurement, and the
     * inverse kinematic values calculated from our ChassisSpeeds (see
     * {@link clampedForwardBack} and friends for details) to determine the PID
     * setpoints.
     */
    private List<PIDController> pivotMotorPIDControllers;

    /**
     * Determines whether or not the shuffle board values should affect the wheels
     * of the robot based on if the joystick is being moved.
     */
    private boolean canShuffleBoardActuate;

    /**
     * The purpose of periodic is to meet these goalStates that we set, whether through
     * a ChassisSpeeds that gets converted to SwerveDriveStates or by the module states
     * directly. 
     */
    private List<SwerveModuleState> goalStates;
    
    /** 
     * A calculator to convert the four individual swerve module angles and
     * speeds to and from a ChassisSpeeds object.
     */
    private SwerveDriveKinematics kinematics;

    /**
     * The velocity that we want the chassis to *eventually* match in the
     * forward/backward axis.  +1.0 is 100% forward, -1.0 is 100% backward.
     *
     * Who is responsible for reaching goal?  Why, {@link periodic()} is, of
     * course!
     */
    private double clampedForwardBack;

    /**
     * The velocity that we want the chassis to *eventually* match in the left/right
     * axis.  +1.0 is 100% right, -1.0 is 100% left.
     */
    private double clampedLeftRight;
    
    /**
     * The angular velocity we want the chassis to (eventually) match as it turns. 
     */
    private double clampedTurn;

    /**
     * Used only for the differential drive.
     */
    private double currentYAxis;

    /**
     * Used only for the differential drive.
     */
    private double currentTurn;

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
                final int BACK_RIGHT = DriveConstants.WheelIndex.BACK_RIGHT.label;
                final int BACK_LEFT = DriveConstants.WheelIndex.BACK_LEFT.label;
                final int FRONT_LEFT = DriveConstants.WheelIndex.FRONT_LEFT.label;
                final int FRONT_RIGHT = DriveConstants.WheelIndex.FRONT_RIGHT.label;

                // Initialize drive motors
                swerveDriveMotors = Arrays.asList(new SparkMax[] {
                    new SparkMax(Constants.DriveConstants.DRIVE_MOTOR_CAN_OFFSET + BACK_RIGHT, MotorType.kBrushless),
                    new SparkMax(Constants.DriveConstants.DRIVE_MOTOR_CAN_OFFSET + BACK_LEFT, MotorType.kBrushless),
                    new SparkMax(Constants.DriveConstants.DRIVE_MOTOR_CAN_OFFSET + FRONT_LEFT, MotorType.kBrushless),
                    new SparkMax(Constants.DriveConstants.DRIVE_MOTOR_CAN_OFFSET + FRONT_RIGHT, MotorType.kBrushless),
                });

                // Initialize pivot motors
                swervePivotMotors = Arrays.asList(new SparkMax[] {
                    new SparkMax(Constants.DriveConstants.PIVOT_MOTOR_CAN_OFFSET + BACK_RIGHT, MotorType.kBrushless),
                    new SparkMax(Constants.DriveConstants.PIVOT_MOTOR_CAN_OFFSET + BACK_LEFT, MotorType.kBrushless),
                    new SparkMax(Constants.DriveConstants.PIVOT_MOTOR_CAN_OFFSET + FRONT_LEFT, MotorType.kBrushless),
                    new SparkMax(Constants.DriveConstants.PIVOT_MOTOR_CAN_OFFSET + FRONT_RIGHT, MotorType.kBrushless),
                });

                // Initialize the pivot motors in a similar manner to how we
                // initialized them for the 2020bot.
                SparkMaxConfig commonConfig = new SparkMaxConfig();
                commonConfig.idleMode(IdleMode.kBrake);
                
                // Apply this config if we want a motor to be inverted
                var invertedConfig = commonConfig.inverted(true);

                // For the motors observed to be operating incorrectly (i.e,
                // going clockwise when instructed to go counterclockwise), this
                // loop will apply an inversion to them. The motors operating
                // correctly will not be changed. 
                for (int i = 0; i < 4; i++) {
                    var motor = swervePivotMotors.get(i);
                    var config = new SparkMaxConfig();
                    config.apply(commonConfig);
                    //TODO: This belongs in drive motor configuration for getting
                    //the drive speed in meters per second.
                    //config.encoder.velocityConversionFactor(getConversionFactor());
                    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                }

                // Initialize CANcoders
                swerveCANCODER = Arrays.asList(new CANcoder[] {
                    new CANcoder(Constants.DriveConstants.PIVOT_MOTOR_CAN_CODER_CAN_ID_OFFSET + BACK_RIGHT),
                    new CANcoder(Constants.DriveConstants.PIVOT_MOTOR_CAN_CODER_CAN_ID_OFFSET + BACK_LEFT),
                    new CANcoder(Constants.DriveConstants.PIVOT_MOTOR_CAN_CODER_CAN_ID_OFFSET + FRONT_LEFT),
                    new CANcoder(Constants.DriveConstants.PIVOT_MOTOR_CAN_CODER_CAN_ID_OFFSET + FRONT_RIGHT),
                });

                pivotMotorPIDControllers = Arrays.asList(new PIDController[] {
                    new PIDController(Constants.DriveConstants.PIVOT_MOTOR_P, Constants.DriveConstants.PIVOT_MOTOR_I, Constants.DriveConstants.PIVOT_MOTOR_D),
                    new PIDController(Constants.DriveConstants.PIVOT_MOTOR_P, Constants.DriveConstants.PIVOT_MOTOR_I, Constants.DriveConstants.PIVOT_MOTOR_D),
                    new PIDController(Constants.DriveConstants.PIVOT_MOTOR_P, Constants.DriveConstants.PIVOT_MOTOR_I, Constants.DriveConstants.PIVOT_MOTOR_D),
                    new PIDController(Constants.DriveConstants.PIVOT_MOTOR_P, Constants.DriveConstants.PIVOT_MOTOR_I, Constants.DriveConstants.PIVOT_MOTOR_D)
                });

                // Set the PID controller's setpoint to the angle of the
                // swerve module state.
                for (int i = 0; i < pivotMotorPIDControllers.size(); i++) {
                    pivotMotorPIDControllers.get(i).setTolerance(Constants.DriveConstants.PIVOT_ANGLE_TOLERANCE_RADIANS);
                    pivotMotorPIDControllers.get(i).enableContinuousInput(0, 2*Math.PI);
                }

                // We only need one SwerveDriveKinematics object for our forward and inverse kinematics
                // calculations, and there are concerns that it may be expensive to make more than one.
                this.kinematics =
                    new SwerveDriveKinematics(Constants.DriveConstants.SWERVE_MODULE_POSITIONS.get(BACK_RIGHT),
                                              Constants.DriveConstants.SWERVE_MODULE_POSITIONS.get(BACK_LEFT),
                                              Constants.DriveConstants.SWERVE_MODULE_POSITIONS.get(FRONT_LEFT),
                                              Constants.DriveConstants.SWERVE_MODULE_POSITIONS.get(FRONT_RIGHT));
                resetToForwardPosition();
                break;
            }
        }
        SmartDashboard.putData(this);
    }

    public void stopAllMotors() {
        if (this.driveType == DriveType.SWERVE_DRIVE) {
            this.swerveDriveMotors.forEach(SparkMax::stopMotor);
            this.swervePivotMotors.forEach(SparkMax::stopMotor);
        }
    }

    /**
     * This function is used for reseting the robot to the 0th position where all angles
     * of the pivot motors are set to their starting value. 
     * 
     * This is most useful for autonomous so that we can code the robot to move
     * based off of those origin angles. It also works well as a testing resource.
     */
    public void resetToForwardPosition() {
        final int BACK_RIGHT = DriveConstants.WheelIndex.BACK_RIGHT.label;
        final int BACK_LEFT = DriveConstants.WheelIndex.BACK_LEFT.label;
        final int FRONT_LEFT = DriveConstants.WheelIndex.FRONT_LEFT.label;
        final int FRONT_RIGHT = DriveConstants.WheelIndex.FRONT_RIGHT.label;

        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(CAN_CODER_ANGLE_OFFSETS[BACK_RIGHT]))),
            new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(CAN_CODER_ANGLE_OFFSETS[BACK_LEFT]))),
            new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(CAN_CODER_ANGLE_OFFSETS[FRONT_LEFT]))),
            new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(CAN_CODER_ANGLE_OFFSETS[FRONT_RIGHT])))};

        goalStates = Arrays.asList(swerveModuleStates);
    }

    /**
     * Returns a factor that we can use to convert an angular speed of the wheels
     * from RPM to meters per second.
     */
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
        //Line 355 is defining a lambda function
        TriConsumer<List<SparkMax>, String, WheelIndex> addMotorHelper = (motors, name, index) -> {
            builder.addDoubleProperty(name,
                                      () -> motors.get(index.label).get(),
                                      (speed) -> {
                                          motors.get(index.label).set(speed);
                                          canShuffleBoardActuate = true;
                                      });
        };

        TriConsumer<List<CANcoder>, String, WheelIndex> addCANcoderHelper = (cancoders, name, index) -> {
            builder.addDoubleProperty(name,
                                      () -> swerveCANCODER.get(index.label)
                                                          .getAbsolutePosition()
                                                          .getValue()
                                                          .in(edu.wpi.first.units.Units.Degrees),
                                      null);
        };

        builder.setSmartDashboardType(this.driveType.toString() == "DIFFERENTIAL_DRIVE" ? "DifferentialDrive" : "SwerveDrive");
        switch (driveType) {
            case DIFFERENTIAL_DRIVE:

                // Add the differential drive motors to the shuffleboard.
                addMotorHelper.accept(differentialDriveMotors, "BR Motor", WheelIndex.BACK_RIGHT);
                addMotorHelper.accept(differentialDriveMotors, "BL Motor", WheelIndex.BACK_LEFT);
                addMotorHelper.accept(differentialDriveMotors, "FL Motor", WheelIndex.FRONT_LEFT);
                addMotorHelper.accept(differentialDriveMotors, "FR Motor", WheelIndex.FRONT_RIGHT);
                break;
            case SWERVE_DRIVE:

                // Add the swerve drive motors to the shuffleboard.
                addMotorHelper.accept(swerveDriveMotors, "BR Drive", WheelIndex.BACK_RIGHT);
                addMotorHelper.accept(swerveDriveMotors, "BL Drive", WheelIndex.BACK_LEFT);
                addMotorHelper.accept(swerveDriveMotors, "FL Drive", WheelIndex.FRONT_LEFT);
                addMotorHelper.accept(swerveDriveMotors, "FR Drive", WheelIndex.FRONT_RIGHT);

                // Add the swerve pivot motors to the shuffleboard.
                addMotorHelper.accept(swervePivotMotors, "BR Pivot", WheelIndex.BACK_RIGHT);
                addMotorHelper.accept(swervePivotMotors, "BL Pivot", WheelIndex.BACK_LEFT);
                addMotorHelper.accept(swervePivotMotors, "FL Pivot", WheelIndex.FRONT_LEFT);
                addMotorHelper.accept(swervePivotMotors, "FR Pivot", WheelIndex.FRONT_RIGHT);

                // Add the cancoders to the shuffleboard.
                addCANcoderHelper.accept(swerveCANCODER, "BR Angle", WheelIndex.BACK_RIGHT);
                addCANcoderHelper.accept(swerveCANCODER, "BL Angle", WheelIndex.BACK_LEFT);
                addCANcoderHelper.accept(swerveCANCODER, "FL Angle", WheelIndex.FRONT_LEFT);
                addCANcoderHelper.accept(swerveCANCODER, "FR Angle", WheelIndex.FRONT_RIGHT);
                
                // Added our goal angles here
                builder.addDoubleProperty("BR Goal Angle", () -> goalStates.get(WheelIndex.BACK_RIGHT.label).angle.getDegrees(), null);
                builder.addDoubleProperty("BL Goal Angle", () -> goalStates.get(WheelIndex.BACK_LEFT.label).angle.getDegrees(), null);
                builder.addDoubleProperty("FL Goal Angle", () -> goalStates.get(WheelIndex.FRONT_LEFT.label).angle.getDegrees(), null);
                builder.addDoubleProperty("FR Goal Angle", () -> goalStates.get(WheelIndex.FRONT_RIGHT.label).angle.getDegrees(), null);

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
     * Drive the robot according to the given speeds.  This is meant to be used
     * during commands.
     *
     * @param forwardBack The speed at which the robot should move forward or
     * backward. A value of 1.0 means full speed forward and -1.0 means full
     * speed backward.
     * @param leftRight The speed at which the robot should move left or right. 1.0
     * should be full speed to the right and -1.0 should be full speed to the
     * left.
     * @param turn The speed at which the robot should turn (1.0 is full speed
     * clockwise and -1.0 is full speed counterclockwise.
     */
    public void drive(double forwardBack, double leftRight, double turn) {
        clampedForwardBack = MathUtil.clamp(forwardBack, -1.0, 1.0);
        clampedLeftRight = MathUtil.clamp(leftRight, -1.0, 1.0); // 
        clampedTurn = MathUtil.clamp(turn, -1.0, 1.0);

        if (driveType == DriveType.SWERVE_DRIVE) {
                
            // Convert the human input into a ChassisSpeeds object giving us
            // the overall bearing of the chassis. The parameters for the ChassisSpeeds are velocities.
            //
            // We will always be driving using values from the drive().        
        
            ChassisSpeeds movement =
                new ChassisSpeeds(Constants.DriveConstants.SWERVE_DRIVE_MAX_DRIVING_SPEED_METERS_PER_SECOND * clampedForwardBack,
                                Constants.DriveConstants.SWERVE_DRIVE_MAX_DRIVING_SPEED_METERS_PER_SECOND * clampedLeftRight,
                                Constants.DriveConstants.SWERVE_DRIVE_MAX_TURNING_SPEED_RADIANS_PER_SECOND * clampedTurn);

            // With inverse kinematics, convert the overall chassis speed
            // into the speeds and angles for all four swerve modules.
            //
            // The .toSwerveModuleStates function is what does inverse kinematics to get 
            // the speed and angle of the individual modules.
            goalStates = Arrays.asList(kinematics.toSwerveModuleStates(movement));
        }
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
                    if (Math.abs(clampedLeftRight) < Constants.MathConstants.EPSILON &&
                        Math.abs(clampedTurn) < Constants.MathConstants.EPSILON) {
                        differentialDrive.arcadeDrive(0.0, 0.0);
                        followDifferentialDrive.arcadeDrive(0.0, 0.0);
                        // System.out.println("stopped.");
                    } else {
                        // Limit the amount that the yAxis and turn values are changed
                        // by calculating the difference between the target value, clampedYAxis
                        // and the current value, currentYAxis. Then clamping the difference to
                        // a threshold and adding the clamped difference to the current value.
                        double diffYAxis = clampedLeftRight - currentYAxis;
                        diffYAxis = MathUtil.clamp(diffYAxis, -0.05, 0.05);
                        currentYAxis += diffYAxis;

                        double diffTurn = clampedTurn - currentTurn;
                        diffTurn = MathUtil.clamp(diffTurn, -0.25, 0.25);
                        currentTurn += diffTurn;

                        differentialDrive.arcadeDrive(currentYAxis, currentTurn);
                        followDifferentialDrive.arcadeDrive(currentYAxis, currentTurn);
                        System.out.println("y-axis: " + clampedLeftRight + " turn: " + clampedTurn);
                    }
                }
                //differentialDriveMotors.get(2).set(0.2);
                //differentialDriveMotors.get(3).set(0.2);

                // differentialDriveMotors.get(0).set(0.2);
                // differentialDriveMotors.get(1).set(0.2);

                break;
            case SWERVE_DRIVE:
                // Our primary input for driving is the goalStates[] that we set in the drive function.

                // Grab CANCoder measurements.
                // Our PID setpoints come from the SwerveModuleStates.
                double[] CANCoderAnglesRadians = new double[4];
                for (int i = 0; i < 4; i++) {
                    var rotations = swerveCANCODER.get(i).getAbsolutePosition(true);
                    // Removed the refresh call because getAbsolutePosition() already refreshes
                    // automatically.
                    // rotations.refresh();
                    CANCoderAnglesRadians[i] = rotations.getValueAsDouble() * 2 * Math.PI;
                    
                    //Uche suggested this but it did not work ):
                    //CANCoderAnglesRadians[i] -= Math.toRadians(CAN_CODER_ANGLE_OFFSETS[i]);
                }

                // TODO: Use the CANCoder's measurements for the PID
                // controllers. 
                //
                // Later, we should InItsendable to send our pivot angles to
                // the suffleboard for easier debugging.

                //
                for (int i = 0; i < 4; i++) {
                    // Get the pivot motor's PID controller.
                    var pivotMotorPIDController = pivotMotorPIDControllers.get(i);

                    // Get the swerve module state.
                    var goalState = goalStates.get(i);

                    // Get the pivot motor.
                    var pivotMotor = swervePivotMotors.get(i);

                    // // Set the PID controller's setpoint to the angle of the
                    // swerve module state.
                    var setpoint = goalState.angle.getRadians();
                    pivotMotorPIDController.setSetpoint(setpoint);

                    if (pivotMotorPIDController.atSetpoint()) {
                        // If the PID controller is at the setpoint, then we
                        // don't need to do anything.
                        pivotMotor.stopMotor();
                    } else {
                        // Get the output from the PID controller.
                        double power = pivotMotorPIDController.calculate(CANCoderAnglesRadians[i],
                                                                         goalState.angle.getRadians());

                        if (isMotorReversed[i])
                            power *= -1.0;
                        // Set the output to the pivot motor.
                        //if(i == 0) {
                        pivotMotor.set(power);
                        if (DriverStation.isTeleopEnabled()) {
                            String[] labels = new String[] {
                                "BR error",
                                "BL error",
                                "FL error",
                                "FR error",
                            };
                            var error = setpoint - CANCoderAnglesRadians[i];
                            SmartDashboard.putNumber(labels[i], error);
                        }
                        String[] labels = new String[] {
                            "BR power",
                            "BL power",
                            "FL power",
                            "FR power",
                        };
                        SmartDashboard.putNumber(labels[i], power);
                        String[] labels2 = new String[] {
                            "BR goal angle",
                            "BL goal angle",
                            "FL goal angle",
                            "FR goal angle",
                        };
                        SmartDashboard.putNumber(labels2[i], goalState.angle.getDegrees());
                    }


                    // We are powering the drive motor without PID because we do
                    // not care when the drive motor reaches a specific velocity
                    // as long it goes vroom.
                    var driveMotor = swerveDriveMotors.get(i);

                    /**
                     * SparkMaxes can only set speeds as percentages. We are
                     * converting the swerveModuleState speed into meters per
                     * second.
                     */
                    final double speed = goalState.speedMetersPerSecond / DriveConstants.SWERVE_DRIVE_MAX_DRIVING_SPEED_METERS_PER_SECOND;

                    // Deadzoning the driving speed to save power.
                    if (Math.abs(speed) < DriveConstants.SWERVE_DRIVE_DEADZONE) {
                        driveMotor.stopMotor();
                    } else {
                        driveMotor.set(speed);
                    }
                }

                break;
            default:
                break;
        }
    }

}
