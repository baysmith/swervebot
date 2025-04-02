package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Reads user input continuously from a wide variety of human interface
 * devices (including, eventually, keyboard input).  The directional inputs
 * are exposed through the three channel querying methods {@link
 * #getForwardBack()}, {@link #getLeftRight()}, and {@link #getTurn()}, while
 * any remaining buttons or sliders are exposed through their own methods.
 *
 * <p>On joystick support:</p>
 * <ul>
 *   <li>
 *     <p>The system requires two joysticks, known as the <em>main</em>
 *     and <em>secondary</em> joysticks. Whichever one was plugged in first
 *     will be main and the most recent one will be secondary.</p>
 *     <ul>
 *       <li> Any joysticks plugged in beyond the first two will be ignored.</li>
 *     </ul>
 *   </li>
 *   <li>
 *     <p>The mappings change as follows:</p>
 *     <ul>
 *       <li>Main joystick, X axis: {@link #getLeftRight() leftRight}</li>
 *       <li>Main joystick, Y axis: {@link #getForwardBack() forwardBack}</li>
 *       <li>Secondary joystick, X axis: {@link #getTurn() turn}</li>
 *     </ul>
 *   </li>
 * </ul>
 * <p>See `README.md` for a complete list of joystick and controller button
 * mappings.</p>
 */
public class InputSubsystem extends SubsystemBase {

    final int CORAL_INTAKE_BUTTON = 1;
    final int ARM_MOVEMENT_UP_BUTTON = 2;
    final int ARM_MOVEMENT_DOWN_BUTTON = 3;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private GenericHID mainController;
    private GenericHID secondaryController;

    /**
     * How often should we look for new human input devices?
     */
    private static final double JOYSTICK_POLLING_INTERVAL_SECONDS = 3.0;

    /**
     * When we last looked for new human input devices, measured in seconds
     * since the robot powered on.
     */
    private double lastCheckTimeSeconds;

    int currentHeight;

    public InputSubsystem() {
        mainController = null;
        secondaryController = null;
        lastCheckTimeSeconds = Timer.getFPGATimestamp() - JOYSTICK_POLLING_INTERVAL_SECONDS; // Force initial check
        currentHeight = 0;
        SmartDashboard.putData(this);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String getName() {
        return "Input Subsystem";
    }

    @Override
    public void periodic() {
        super.periodic();

        // Check to see if anything new was plugged in (or removed.)
        double currentTimeSeconds = Timer.getFPGATimestamp();
        if (currentTimeSeconds - lastCheckTimeSeconds >= JOYSTICK_POLLING_INTERVAL_SECONDS) {
            controllerCheck();
            lastCheckTimeSeconds = currentTimeSeconds;
        }
    }

    /**
     * How much does the user want to strafe sideways?  -1.0 is full speed
     * leftward, +1.0 is full speed rightward, and 0.0 means that no strafing
     * should take place.
     *
     * <ul>
     *   <li>On the Xbox controller, this is the left joystick's horizontal channel.</li>
     *   <li>For one-joystick setups, this is the horizontal channel.</li>
     *   <li>For two-joystick setups, this is the horizontal channel of the
     *   primary (first) joystick.</li>
     *   <li>For keyboard input, this is controlled by the A and D keys.</li>
     * </ul>
     */
    public double getLeftRight() {
        double joystickLeftRight = mainController != null ? mainController.getRawAxis(0) : 0;

        // If all devices are plugged in, then they'll all contribute so long as
        // a human is touching them.  This allows for pair driving.
        double leftRight = MathUtil.applyDeadband(joystickLeftRight, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE);

        leftRight = MathUtil.clamp(leftRight, -1.0, 1.0);

        return m_xspeedLimiter.calculate(leftRight);
    }

    /**
     * How much does the user want to drive straight ahead?  -1.0 is full speed
     * backward, +1.0 is full speed forward, and 0.0 means that no movement
     * should occur in that axis.
     *
     * <ul>
     *   <li>On the Xbox controller, this is the left joystick's vertical channel.</li>
     *   <li>For one-joystick setups, this is the vertical channel.</li>
     *   <li>For two-joystick setups, this is the vertical channel of the
     *   primary (first) joystick.</li>
     *   <li>For keyboard input, this is controlled by the W and S keys.</li>
     * </ul>
     */
    public double getForwardBack() {
        double joystickForwardBack = mainController != null ? mainController.getRawAxis(1) : 0;

        double forwardBack = MathUtil.applyDeadband(joystickForwardBack, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE);

        forwardBack = MathUtil.clamp(forwardBack, -1.0, 1.0);
        
        return m_yspeedLimiter.calculate(forwardBack);
    }

    /**
     * How much does the user want to turn?  -1.0 is full speed
     * counterclockwise, +1.0 is full speed clockwise, and 0.0 means not to
     * turn.
     *
     * <ul>
     *   <li>On the Xbox controller, this is the right joystick's horizontal channel.</li>
     *   <li>For one-joystick setups, this is the so-called "Z" channel
     *   (produced by twisting the joystick.)</li>
     *   <li>For two-joystick setups, this is the horizontal channel of the
     *   secondary joystick.</li>
     *   <li>For keyboard input, this is controlled by the LEFT and RIGHT keys.</li>
     * </ul>
     */
    public double getTurn() {
        double joystickTurn = secondaryController != null ? secondaryController.getRawAxis(0) : (
                               mainController != null ? mainController.getRawAxis(2) : 0);

        double turn = MathUtil.applyDeadband(joystickTurn, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE);

        turn = MathUtil.clamp(turn, -1.0, 1.0);

        return m_rotLimiter.calculate(turn);
    }

    /**
     * THIS IS OBSOLETE. WE WILL NOT BE USING THIS AT COMP.
     * A function to return the number of desired rotations of the lift motors based on user input.
     * - Stage 0 represents base height of the lift when idle, which is equal to the height of the trough.
     * - Each additional stage (up to the top) represents the next level of the coral branch.
     *   - Lift stage 1 -> coral level 2
     *   - Lift stage 2 ->  coral level 3
     *   - Lift stage 3 -> coral level 4
     *
     * - Each time the user presses one of the lift triggering buttons,
     *   the lift will automatically transition between the stages.
     */
    public double getDesiredPosition() {
        double[] liftHeights = {
            Constants.ArmConstants.DEFAULT_HEIGHT,
            Constants.ArmConstants.LIFT_HEIGHT_1,
            Constants.ArmConstants.LIFT_HEIGHT_2,
            Constants.ArmConstants.LIFT_HEIGHT_3,
        };

        if (secondaryController != null) {
            if (secondaryController.getRawButtonPressed(ARM_MOVEMENT_UP_BUTTON) && currentHeight < liftHeights.length - 1) {
                currentHeight += 1;
            } else if (secondaryController.getRawButtonPressed(ARM_MOVEMENT_DOWN_BUTTON) && currentHeight > 0) {
                currentHeight -= 1;
            }
        }
        return liftHeights[currentHeight];
    }

    /**
     * This is a separate method of controlling the lift of the ArmSubsystem.
     * It will move the lift at a constant speed as long as the player is holding down the desired movement button.
     * The speed of the lift is dependent on the constant LIFT_SPEED in Constants.java.
     */
    public double getArmMovement() {
        if (mainController != null) {
            if (mainController.getRawButton(ARM_MOVEMENT_UP_BUTTON)) {
                return Constants.ArmConstants.LIFT_SPEED;
            } else if (mainController.getRawButton(ARM_MOVEMENT_DOWN_BUTTON)) {
                return Constants.ArmConstants.LIFT_SPEED * -1.0;
            }
        }
        return 0.0;
    }


    /**
     * Returns whether or not the coral flywheels should be moving based on whether or not the player
     * is pressing the right trigger/right bumper button.
     */
    public boolean isCoralIntakeActivated() {
        if (mainController != null && mainController.getRawButton(CORAL_INTAKE_BUTTON)) {
            return true;
        }
        return false;
    }


    /**
     * A function called every few seconds meant to detect when xbox controllers
     * or joysticks are connected or disconnected, and updates dynamically.
     *
     * <p>To avoid overwhelming the system with exceptions, callers of this
     * function should only do so occasionally.</p>
     */
    private void controllerCheck() {
        if (mainController != null) {
            if (!mainController.isConnected()) {
                System.out.println("Main controller disconnected.");
                mainController = null;
            }
        }
        if (secondaryController != null) {
            if (!secondaryController.isConnected()) {
                System.out.println("Secondary controller disconnected.");
                secondaryController = null;
            }
        }

        final int MAX_PORTS_TO_CHECK = 5;
        if (mainController == null) {
            for (int port = 0; port < MAX_PORTS_TO_CHECK; port++) {
                try {
                    var xboxController = new XboxController(port);
                    if (xboxController.isConnected()) {
                        mainController = xboxController;
                        System.out.printf("Main controller connected (xbox, port %d).\n", mainController.getPort());
                        break;
                    }
                } catch (Exception e) {
                    System.out.printf("xbox controller, port %d, exception %s\n", port, e.toString());
                }

                try {
                    Joystick j = new Joystick(port);
                    if (j.isConnected()) {
                        mainController = j;
                        System.out.printf("Main controller connected (joystick, port %d).\n", mainController.getPort());
                        break;
                    }
                } catch (Exception e) {
                    System.out.printf("joystick, port %d, exception %s\n", port, e.toString());
                }
            }
        }
        if (mainController != null && secondaryController == null) {
            for (int port = 0; port < MAX_PORTS_TO_CHECK; port++) {
                if (mainController.getPort() == port)
                    continue;
                try {
                    Joystick j = new Joystick(port);
                    if (j.isConnected()) {
                        secondaryController = j;
                        System.out.printf("Secondary controller connected (joystick, port %d).\n", port);
                        break;
                    }
                } catch (Exception e) {
                    System.out.printf("joystick, port %d, exception %s\n", port, e.toString());
                }
            }
        }
    }

    /**
     * A simple utility function that returns true if {@link #mainController} is
     * usable.
     */
    private boolean mainJoystickFound() {
        return mainController != null && mainController.isConnected();
    }

    /**
     * A simple utility function that returns true if {@link
     * #secondaryController} is usable.
     */
    private boolean secondaryJoystickFound() {
        return secondaryController != null && secondaryController.isConnected();
    }

    /**
     * Writes the input values to the shuffleboard.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("InputSubsystem");
        builder.addDoubleProperty("Turn", this::getTurn, null);
        builder.addDoubleProperty("ForwardBack", this::getForwardBack, null);
        builder.addDoubleProperty("LeftRight", this::getLeftRight, null);
        builder.addBooleanProperty("CoralIntake", this::isCoralIntakeActivated, null);
        builder.addDoubleProperty("ArmMovement", this::getArmMovement, null);
        builder.addDoubleProperty("LiftPosition", this::getDesiredPosition, null);
        // super.initSendable(builder);
    }

    // TODO We need to call the controller check every few seconds during
    // periodic, and we need to combine the xbox controller, the main joystick,
    // and the secondary joystick into the 3 degrees of freedom: forward-back,
    // left-right, and turn.
}
