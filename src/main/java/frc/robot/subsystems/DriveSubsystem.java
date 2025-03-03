package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BiConsumer;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Constants.DriveConstants.DriveType;
import frc.robot.Constants.DriveConstants.WheelIndex;

public class DriveSubsystem extends SubsystemBase {

    private InputSubsystem input;
    private double maxDriveSpeed;
    private double maxTurnSpeed;
    private DriveType driveType;
    private DifferentialDrive differentialDrive;

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
                final int FRONT_LEFT = DriveConstants.DRIVE_MOTOR_CAN_OFFSET + DriveConstants.WheelIndex.FRONT_LEFT.label;
                final int FRONT_RIGHT = DriveConstants.DRIVE_MOTOR_CAN_OFFSET + DriveConstants.WheelIndex.FRONT_RIGHT.label;
                final int BACK_RIGHT = DriveConstants.DRIVE_MOTOR_CAN_OFFSET + DriveConstants.WheelIndex.BACK_RIGHT.label;
                final int BACK_LEFT = DriveConstants.DRIVE_MOTOR_CAN_OFFSET + DriveConstants.WheelIndex.BACK_LEFT.label;
                differentialDriveMotors = Arrays.asList(new SparkMax[] {
                    new SparkMax(FRONT_RIGHT, MotorType.kBrushed),
                    new SparkMax(BACK_RIGHT, MotorType.kBrushed),
                    new SparkMax(BACK_LEFT, MotorType.kBrushed),
                    new SparkMax(FRONT_LEFT, MotorType.kBrushed)
                });

                // We are using two different Configs for the left and right
                // motors as sometimes they will be doing different things at
                // times. For example, when turning, the left and the right side
                // will be moving in the opposite directions.
                SparkMaxConfig commonConfig = new SparkMaxConfig();
                commonConfig.idleMode(IdleMode.kBrake);
                SparkMaxConfig leftConfig = new SparkMaxConfig();
                SparkMaxConfig rightConfig = new SparkMaxConfig();
                leftConfig.follow(FRONT_LEFT).apply(commonConfig);
                rightConfig.follow(FRONT_RIGHT).apply(commonConfig);
                differentialDriveMotors.get(FRONT_LEFT).configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                differentialDriveMotors.get(BACK_LEFT).configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                differentialDriveMotors.get(FRONT_RIGHT).configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                differentialDriveMotors.get(BACK_RIGHT).configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                differentialDrive = new DifferentialDrive(differentialDriveMotors.get(FRONT_LEFT),
                                                          differentialDriveMotors.get(FRONT_RIGHT));
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
                break;
            }
        }
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
                break;
        }
        builder.setActuator(true);
        builder.setSafeState(this::disable);
        initSendable(builder);
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
                if (input.getForwardBack() != 0 || input.getTurn() != 0) {
                    differentialDrive.arcadeDrive(input.getForwardBack(), input.getTurn());
                    canShuffleBoardActuate = false;
                } else if (!canShuffleBoardActuate) { 
                    differentialDrive.arcadeDrive(0, 0);
                }
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

                SwerveModuleState[] swerveStates = kinematics.toSwerveModuleStates(movement);

                double[] CANCoderAngles = new double[4];
                for (int i = 0; i < 4; i++) {
                   var temporary = swerveCANCODER.get(i).getAbsolutePosition();
                   temporary.refresh();
                   CANCoderAngles[i] = temporary.getValueAsDouble();
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

}
