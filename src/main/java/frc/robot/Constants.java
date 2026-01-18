// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    /**
     * Dead zone for the joystick -- or, better to say, for the
     * {@link SlewRateLimiter} that we associate with each joystick.
     *
     * Any joystick inputs with absolute values less than this will be treated
     * as if they are 0.
     */
    public static final double JOYSTICK_DEAD_ZONE = 0.02;
  }

  public static class DriveConstants {
    /**
     * The radius of your swerve wheels.  Used in calculations to determine
     * velocity of each module in meters per second.
     *
     * <p>TODO: Measure the real value for this number,</p>
     */
    public static final double SWERVE_MODULE_WHEEL_RADIUS_METERS = 0.0508;

    /**
     * Theoretical maximum of chassis speed in meters per second for kinematic
     * conversion purposes.
     */
    public static final double SWERVE_DRIVE_MAX_DRIVING_SPEED_METERS_PER_SECOND = 0.5;

    /**
     * SparkMaxes set velocities in abstract percentages [-1,1]. As such, the
     * deadzone is a percentage too.
     */
    public static final double SWERVE_DRIVE_DEADZONE = 0.01;

    /**
     * Theoretical maximum turning speed of the robot in radians per second for
     * kinematic conversion purposes.
     */
    public static final double SWERVE_DRIVE_MAX_TURNING_SPEED_RADIANS_PER_SECOND = 0.4;

    /**
     * The horizontal distance between the left and the right swerve modules in meters. 
     * This is measured from the center of the wheels.
     */
    private static final double CHASSIS_WIDTH_METERS = 0.495;

    /**
     * The distance between the front and the back swerve modules in meters.
     * This is measured from the center of the wheels.
     */
    private static final double CHASSIS_LENGTH_METERS = 0.495; 

    /**
     * The positions of the wheels on the Chassis measured in meters.
     *
     * {@link ChassisSpeeds} class considers the y-axis to represent the
     * side-to-side movement, which holonomic drives like ours have but
     * differential drives do not.
     *
     * @todo Measure the real values for these numbers.
     */
    public static final List<Translation2d> SWERVE_MODULE_POSITIONS = Arrays.asList(
      new Translation2d(-CHASSIS_LENGTH_METERS / 2, CHASSIS_WIDTH_METERS / 2),  // BACK RIGHT
      new Translation2d(-CHASSIS_LENGTH_METERS / 2, -CHASSIS_WIDTH_METERS / 2), // BACK LEFT
      new Translation2d(CHASSIS_LENGTH_METERS / 2, -CHASSIS_WIDTH_METERS / 2),  // FRONT_LEFT
      new Translation2d(CHASSIS_LENGTH_METERS / 2, CHASSIS_WIDTH_METERS / 2)    // FRONT_RIGHT
    );

    /**
     * The deadzone for the pivot angles. As the PID controller will try to
     * reach its setpoint, it will consider values within this range close
     * enough.
     *
     * TODO: Test and see if this value is too small or too large.
     */
    public static final double PIVOT_ANGLE_TOLERANCE_RADIANS = Math.PI / 180;

    /**
     * The type of drive that the {@link DriveSubsystem} will use.
     */
    public enum DriveType {
      SWERVE_DRIVE,
      DIFFERENTIAL_DRIVE
    }

    /**
     * <p> We will have an array of four pivot motors and four drive motors. This
     * enumeration is indices for the arrays so that we know which drive motor
     * is in what location. </p>
     *
     * <p> If you see a shuffleboard display coming from PivotMotor2, then you will
     * know you are dealing with a BACK_RIGHT pivot motor. </p>
     *
     * <p> This list will be null if we are using a differential drive. </p>
     */
    public enum WheelIndex {
        FRONT_LEFT(2),
        FRONT_RIGHT(3),
        BACK_RIGHT(0),
        BACK_LEFT(1);

        public final int label;

        private WheelIndex(int label) {
            this.label = label;
        }
    }

    // The differential drive does not care about the orders of the left and
    // right motors because the same gearbox is driving the entire right and
    // left side.
    public static final int DIFFERENTIAL_DRIVE_PWM_LEFT_MOTOR_1 = WheelIndex.FRONT_LEFT.label;
    public static final int DIFFERENTIAL_DRIVE_PWM_LEFT_MOTOR_2 = WheelIndex.BACK_LEFT.label;
    public static final int DIFFERENTIAL_DRIVE_PWM_RIGHT_MOTOR_1 = WheelIndex.FRONT_RIGHT.label;
    public static final int DIFFERENTIAL_DRIVE_PWM_RIGHT_MOTOR_2 = WheelIndex.BACK_RIGHT.label;

    /**
     * <p> Two motors cannot have the same ID on the CANBus; however, we still want
     * to use the WheelIndex enumeration as it provides us with an easy way to
     * know which motor is which. However, in a swerve drive, two motors are on
     * the same module, and that creates a conflict. </p>
     *
     * <p> To resolve the conflict, we will offset the IDs of all PivotMotors by the
     * number of drive motors. </p>
     *
     * <p> Suppose you have a random CAN ID. If it is between 0 and 3, it is a drive
     * motor. If it is between 4 and 7, it is a pivot motors. All other values
     * are neither. </p>
     *
     *
     * </p> To find the position of the CAN ID on the chassis, take CAN ID modulo 4. </p>
     */
    public static final int PIVOT_MOTOR_CAN_OFFSET = 5;
    public static final int DRIVE_MOTOR_CAN_OFFSET = 1;

    /**
     * Assumption: IF we can assign the CAN IDs to all four CANCoders, *THEN* we prefer to
     * assign them in the order:
     * <ol>
     *    <li>FRONT_LEFT</li> 11
     *    <li>FRONT_RIGHT</li> 12
     *    <li>BACK_RIGHT</li> 9
     *    <li>BACK_LEFT</li> 10
     *  </ol>
     * This will give us a way to predict the correct CANCoder behind any CAN ID
     * we see.
     *
     */
    public static final int PIVOT_MOTOR_CAN_CODER_CAN_ID_OFFSET = 9;

    /**
     * PID Constants for the Pivot Motors
     * TODO: Find the PID Constants for the Pivot Motors.
     */
    public static final double PIVOT_MOTOR_P = 0.05;
    public static final double PIVOT_MOTOR_I = 0;
    public static final double PIVOT_MOTOR_D = 0;



  }

  public static class ArmConstants {

    /******************************
    * CAN IDS FOR THE ARM MOTORS *
    ******************************/

    // The Pivot Motor CAN IDs are between 9 and 12. As such, we set the arm
    // motor IDs to very large values so we will have a large buffer between the
    // CAN IDs; this way, we can avoid any conflicts.
    public static final int LEFT_LIFT_CAN_ID = 20;
    public static final int RIGHT_LIFT_CAN_ID = 21;
    public static final int LEFT_CORAL_CAN_ID = 22;
    public static final int RIGHT_CORAL_CAN_ID = 23;

    /**
     * The heights of each stage will be in terms of rotations, which need to updated with testing.
     */
    public static final double DEFAULT_HEIGHT = 0;
    public static final double LIFT_HEIGHT_1 = 1;
    public static final double LIFT_HEIGHT_2 = 2;
    public static final double LIFT_HEIGHT_3 = 3;

    public static final double LIFT_SPEED = 0.17; //originally 0.12
    public static final double CORAL_OUTTAKE_SPEED = 0.1;
  }

  public static class MathConstants {
    public static final double EPSILON = 1e-5;
  }
}
