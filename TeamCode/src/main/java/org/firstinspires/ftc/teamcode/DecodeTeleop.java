
//    This is a complete DECODE Teleop demonstration program.
//    It's meant as a Proof of Concept, that the AprilTags in this year's game are useful, even in Teleop.
//    It's based on the RobotAutoDriveToAprilTagOmni sample program.
//    Changes from the sample are marked with a comment with DECODE as a keyword.
//    Having already picked up a ball, the driver needs to turn and face a goal.
//    They then press the B button to have the robot drive to position and launch a ball at the Red goal.
//    Or they can press the X button to have the robot drive to position and launch a ball at the Blue goal.
//    The Y button raise the intake, assuming there is a ball in it.
//    The A button lowers the intake, ready to collect another ball.
//    The right bumper will manually launch an already collected ball (assuming the intake was raised).

//    Disclaimer: This code is not meant to be used as is. It was only developed enough for the proof of concept.

//    Teleop programs are often written as iterative opModes, as the natural structure of an iterative opMode
//    is the loop() method where the programs checks for gamepad input or changes in sensors and does an action
//    and then loops again. In this style of programming, you need to remember what your robot is trying to do
//    during repeated loops.
//
//    This can be done by using State Machine logic. We consider the robot to be in a particular 'state'
//    and the robot stays in that state until some input or change requires that the robot change to a new state.
//
//    The starting source for this program was the RobotAutoDriveToAprilTagOmni.java program.
//    It's a linear opMode program, which doesn't use loop(), but that means we have to stay in the While
//    loop waiting for the OpMode to be stopped, but otherwise responding to gamepad input or checking
//    sensor values (like motor encoders) to know when to change state.

/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: front_left_drive and front_right_drive, back_left_drive and back_right_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID.
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@TeleOp

public class DecodeTeleop extends LinearOpMode
{
    // DECODE Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 48.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.02 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor frontLeftDrive = null;  //  Used to control the left front drive wheel
    private DcMotor frontRightDrive = null;  //  Used to control the right front drive wheel
    private DcMotor backLeftDrive = null;  //  Used to control the left back drive wheel
    private DcMotor backRightDrive = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    public enum RobotState {    // DECODE: we'll use a Java enum to represent our robot states
        TELEOP("Teleop"),       // the robot will respond to joystick driving
        INTAKE("Intake"),       // the intake is moving
        REDGOAL("RedGoal"),     // AprilTag drive to a Red goal launch area
        BLUEGOAL("BlueGoal"),   // AprilTag drive to a Blue goal launch area
        LAUNCH("Launch"),       // launch a ball from the launch area
        FLYWHEEL("Flywheel"),   // spin up the flywheel
        FEED("Feed"),           // feed the ball into the flywheel
        ENDLAUNCH("EndLaunch"); // after launching, turn off flywheel and feed motors, then return to TELEOP.
        
        private final String state;

        RobotState(String state) {
            this.state = state;
        }

        public String getName() {
            return state;
        }
    }
    // DECODE: additional hardware
    private DcMotor         intake   = null;
    private DcMotor         feed   = null;
    private DcMotor         flywheel  = null;
    private NormalizedColorSensor colorSensor;
    
    // DECODE: these variables moved here from runOpMode to make them global as we moved the AprilTag driving logic into it's own method.
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;        // Desired turning power/speed (-1 to +1)
    
            /*  This started as the RobotAutoDriveToAprilTagOmni sample program.
            The basic structure of the program is kept, but we need to add DECODE mechanisms.
            The original while loop is focused on AprilTag driving.
            We still want to do that, but only when we want to line up to launch.
            We also want to pick up balls and launch them.
            As a LinearOpMode we need to control all of this inside the While loop.
            We also can't go to sleep for a long time or wait for something because
            we need the while loop to keep executing. It checks if STOP was pressed and
            allows background process to get sensor input and run motors.
            We'll use a robotState variable to control the robot.
            We start in the TELEOP state for joystick based Teleop driving.
        */
        RobotState robotState = RobotState.TELEOP;
        
    @Override public void runOpMode()
    {
        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "drive_leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "drive_rightFront");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "drive_leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "drive_rightBack");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
            
        // DECODE: additional hardware init
        intake  = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feed  = hardwareMap.get(DcMotor.class, "feed");
        feed.setDirection(DcMotor.Direction.FORWARD);
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // setVelocity uses an angular rate specified as ticks per second
        // the flywheel is a HD Hex Motor (REV-41-1291) with no gearbox
        // with 28 ticks per rotation of the shaft.
        double angularRate = 2000;  // 2800 is more than this motor can do
        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        // Note: we coud detect purple or green, but only using the distance feature of this sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        ElapsedTime     runtime = new ElapsedTime();
        
        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();
        
        while (opModeIsActive())
        {

            switch(robotState) {
                case TELEOP:
                    if (gamepad1.b) {
                        robotState = RobotState.REDGOAL;
                    }
                    if (gamepad1.x) {
                        robotState = RobotState.BLUEGOAL;
                    }
                    if (gamepad1.y) {   // raise intake
                        intake.setTargetPosition(625);
                        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.setPower(0.3);
                        robotState = RobotState.INTAKE;
                    }
                    if (gamepad1.a) {   // lower intake
                        intake.setTargetPosition(0);
                        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.setPower(0.3);
                        robotState = RobotState.INTAKE;
                    }
                    if (gamepad1.right_bumper) {   // manual launch
                        robotState = RobotState.LAUNCH;
                    }
                    // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                    drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
                    strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                    turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                    telemetry.addData("TELEOP","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                    // Apply desired axes motions to the drivetrain.
                    moveRobot(drive, strafe, turn);
                    sleep(10);                    
                    break;
                case INTAKE:    // we're waiting for the intake to finish moving
                    if (!intake.isBusy()) {
                        intake.setPower(0);
                        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robotState = RobotState.TELEOP;  // back to TELEOP when intake done
                    }
                    break;
                case BLUEGOAL:
                    // Drive toward the Blue goal (AprilTag ID 20)
                    aprilTagDrive(20);  // this method sets the drive variables.
                    telemetry.addData("BLUEGOAL","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                    // Apply desired axes motions to the drivetrain.
                    moveRobot(drive, strafe, turn);
                    sleep(10);
                    break;
                case REDGOAL:
                    // Drive toward the Red goal (AprilTag ID 24)
                    aprilTagDrive(24);  // this method sets the drive variables.
                    telemetry.addData("REDGOAL","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                    // Apply desired axes motions to the drivetrain.
                    moveRobot(drive, strafe, turn);
                    sleep(10);
                    break;
                case LAUNCH:
                    telemetry.addData("LAUNCH","initiate launch");
                    moveRobot(0, 0, 0); // ensure robot is not moving
                    // Start the flywheel
                    ((DcMotorEx) flywheel).setVelocity(angularRate);
                    robotState = RobotState.FLYWHEEL;
                    break;
                case FLYWHEEL:
                    telemetry.addData("FLYWHEEL","spin up flywheel");
                    if (((DcMotorEx) flywheel).getVelocity() >= angularRate) {
                        // spin up complete
                        robotState = RobotState.FEED;
                    }
                    break;
                case FEED:
                    telemetry.addData("FEED","feed ball");
                    feed.setPower(1.0); // start to feed the ball
                    runtime.reset();    // start timer
                    robotState = RobotState.ENDLAUNCH;
                    break;
                case ENDLAUNCH:
                    if (runtime.milliseconds() > 1500) {    // wait 1.5 second
                        telemetry.addData("ENDLAUNCH","stopping motors");
                        feed.setPower(0);
                        flywheel.setPower(0);
                        robotState = RobotState.TELEOP;
                    }
                    break;
                default:
                    telemetry.addData("Bad State","Unexpected Robot State");
                    moveRobot(0, 0, 0); // stop robot if driving
            }
            telemetry.update();
        }
    }

    public void aprilTagDrive(int targetTag) {
            targetFound = false;
            desiredTag  = null;
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == targetTag) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                /*telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");*/
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } /*else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            } */

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (/*gamepad1.left_bumper && */ targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;
                if ((Math.abs(rangeError) < 5) && 
                        (Math.abs(headingError) < 5) &&
                        (Math.abs(yawError) < 5)) {     // DECODE test if we are "close enough"
                    robotState = robotState.LAUNCH;     // if we are, change to the LAUNCH state.
                }
                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                //telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
    }
    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
