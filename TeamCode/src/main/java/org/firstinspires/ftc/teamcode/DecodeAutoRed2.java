/*
    This is a complete DECODE autonomous demonstration program.
    It's based on the RobotAutoDriveToAprilTagOmni sample program.
    Changes from the sample are marked with a comment with DECODE as a keyword.
*/

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

import android.graphics.Color;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
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
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
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
 *625
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous
//@Disabled
public class DecodeAutoRed2 extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    /*final*/ double DESIRED_DISTANCE = 48.0; //  this is how close the camera should get to the target (inches)
    final double DESIRED_BEARING = 0.0;   //  this is how close the camera should get to the target (inches)
    /*final*/ double DESIRED_YAW = 0.0;   //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.02 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.015  ;   // (was 0.01) Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    
    // DECODE - additional hardware/sensors
    private DcMotor flywheel = null;    // DECODE - flywheel motor
    private DcMotor intake = null;      // DECODE - motor to raise intake with ball
    private DcMotor feed = null;        // DECODE - motor to feed a ball into the flywheel
    IMU imu;                                    // The IMU sensor object
    
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 24;     // Choose the tag you want to approach or set to -1 for ANY tag.
                         // TAG_ID 24 is the RED goal, Tag 20 is the BLUE goal
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        boolean inPosition     = false;    // Set to true when we're ready to launch ball
    
    ColorBlobLocatorProcessor colorLocator; //DECODE - detect coloured balls
    private WebcamName webcam1, webcam2;
    
    @Override public void runOpMode()
    {
        // DECODE: setVelocity uses an angular rate specified as ticks per second
        // the flywheel is a HD Hex Motor (REV-41-1291) with no gearbox
        // with 28 ticks per rotation of the shaft.
        double angularRate = 1800;  //Note: this number chosen by trial and error.

        // Initialize the Apriltag Detection process
        telemetry.addData("initilize", "initAprilTag");
        telemetry.update();
        initAprilTag();
        
        // initialize the ColorBlobLocatorProcessor
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
        
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
            .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);
            
        visionPortal = new VisionPortal.Builder()
                    .setCamera(switchableCamera)
                    .addProcessor(aprilTag)
                    .addProcessor(colorLocator)
                    .build();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "drive_leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "drive_rightFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "drive_leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "drive_rightBack");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // DECODE - initialize the other motors
        feed  = hardwareMap.get(DcMotor.class, "feed");
        feed.setDirection(DcMotor.Direction.FORWARD);
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        intake  = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // raise intake with pre-loaded ball
        telemetry.addData("initilize", "raise intake");
        telemetry.update();
        intake.setTargetPosition(650);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(0.3);
        while (intake.isBusy()) {
            sleep(10);  // wait for intake to be raised
        }
        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // DECODE - copy in IMU handling from the SensorIMUOrthoganol.java sample program
        telemetry.addData("initilize", "init IMU");
        telemetry.update();
        imu = hardwareMap.get(IMU.class, "imu");
        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode, target = tag 24 (RED)");
        telemetry.update();
        visionPortal.setActiveCamera(webcam1);
        waitForStart();

        // DECODE we moved the AprilTag driving logic into it's own method
        while (opModeIsActive() && !inPosition)
        {
            aprilTagDrive();    // Drive up to the red goal
        }
        // DECODE - exit the while loop after driving to target position, or robot stopped
        moveRobot(0,0,0); // ensure we stop moving
        
        if (opModeIsActive()) { // check if still active
            telemetry.addData("\n>","STEP 2 launch ball \n");
            telemetry.update();  
            ((DcMotorEx) flywheel).setVelocity(angularRate);
        }
        
        // DECODE - if still active, spin up the flywheel
        while (opModeIsActive() && (((DcMotorEx) flywheel).getVelocity() < angularRate)) {
                telemetry.addData("Program", "Flywheel running");
                telemetry.addData("Target",  " at %7.0f", angularRate);
                telemetry.addData("Currently at",  " at %7.0f",
                                            ((DcMotorEx) flywheel).getVelocity());
                telemetry.update();
        }
        if(opModeIsActive()) {          // just in case STOP was pressed
            feed.setPower(1.0);         // turn on feed motor
            sleep(1500);                // wait for the ball to be fed and launched
        }
        // stop both motors
        feed.setPower(0);
        flywheel.setPower(0);
        
        // reposition on the middle ball set
        inPosition = false;
        DESIRED_DISTANCE = 66.0; 
        DESIRED_YAW = -35.0;
        while (opModeIsActive() && !inPosition)
        {
            aprilTagDrive();    // move back and strafe right
        }
        visionPortal.setActiveCamera(webcam2); // use the cam that looks down
        //sleep(2000);    // pause just to check position
        
        // turn to face the red alliance wall and the PPG ball stack
        if (opModeIsActive()) { // check if still active
            imuTurn(-90);
        }
        
        // lower the intake
        if (opModeIsActive()) { // check if still active
            intake.setTargetPosition(0);
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setPower(0.3);
            while (intake.isBusy()) {
                sleep(10);  // wait for intake to be lowered
            }
            intake.setPower(0);
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        
        // use the colored ball locator to see if we need to re-align
        if (opModeIsActive()) { // check if still active
            colourDrive(270);
        }
        // Note: This does not work reliably for this robot. The collector is too narrow, so lining up has to be very accurate.
        // If the robot had a wider intake that would make this easier.
        // In fact, with a wider intake I might not use colourDrive, probably just imuDive forward over the balls collecting them.
        
        // drive forward to collect ball, hold on heading -90
        if (opModeIsActive()) { // check if still active
            imuDrive(-90,1250);  // -90 heading, 1250 ms elapsed time
        }
        
        // TODO: 
        // - check color sensor (distance mode) for ball in intake
        // - raise intake
        // - IMU turn to face red goal april tag
        // - AprilTag drive to launch position
        // - launch ball
        // done.
        
    }

    /* DECODE - add an imuTurn() method that turns the robot to a given heading. */
    public void imuTurn(double targetHeading) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentHeading   = orientation.getYaw(AngleUnit.DEGREES);
        double headingError     = currentHeading - targetHeading;
        double turn;
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();    
        // add runtime and stop trying to turn if 1 second has elapsed
        // this can happen when the calculated motor power values are not enough to overcome static friction
        
        while (opModeIsActive() && (Math.abs(headingError) > 4) && (runtime.milliseconds() < 1000)) { 
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            telemetry.addData("Program", "imuTurn");
            telemetry.addData("Target heading",  " at %4.0f", targetHeading);
            telemetry.addData("Current heading",  " at %4.0f", currentHeading);
            telemetry.update();
            // Apply desired axes motions to the drivetrain.
            moveRobot(0, 0, -turn);  // we only want to rotate
            sleep(10);  // apply power for a little bit, then get new values.
            orientation     = imu.getRobotYawPitchRollAngles();
            currentHeading  = orientation.getYaw(AngleUnit.DEGREES);
            headingError    = currentHeading - targetHeading;
        }
    }
    
    /* DECODE - add an imuDrive() method that drive by time on a heading. */
    public void imuDrive(double targetHeading, int timeMs) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentHeading   = orientation.getYaw(AngleUnit.DEGREES);
        double headingError     = currentHeading - targetHeading;
        double turn;
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        
        while (opModeIsActive() && (runtime.milliseconds() < timeMs) ) { 
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            telemetry.addData("Program", "imuTurn");
            telemetry.addData("Target heading",  " at %4.0f", targetHeading);
            telemetry.addData("Current heading",  " at %4.0f", currentHeading);
            telemetry.update();
            // Apply desired axes motions to the drivetrain.
            moveRobot(0.25, 0, -turn);  // 0.25 drive forward with turn to keep heading.
            sleep(10);  // apply power for a little bit, then get new values.
            orientation     = imu.getRobotYawPitchRollAngles();
            currentHeading  = orientation.getYaw(AngleUnit.DEGREES);
            headingError    = currentHeading - targetHeading;
        }
    }

    /* DECODE - colorDrive() method that lines up on a ball on the field. 
              - The idea is to look for the X value of the color blob.
              - By using the sample color locator program I found a target X value of 270.
                This is the x coorinate of the ball on the image.
              - By comparing the current x coordinate to the target we determine how much 
                we need to move left/right to line move the robot so the ball is at the target location. */
                
    public void colourDrive(int targetX) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentHeading   = orientation.getYaw(AngleUnit.DEGREES);
        double targetHeading    = -90;  // want to continue pointing at the Red wall
        double headingError     = currentHeading - targetHeading;
        int xError;
        int currentX = 0;
        double turn, strafe;

        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, blobs);  // filter out very small blobs.
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.6, 1, blobs);     // filter out non-circular blobs.
        // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            if (currentX == 0) {
                Circle circleFit = b.getCircle();
                currentX = (int) circleFit.getX();
            }
        }
        if (currentX == 0) {    // guarding against no circle found
            xError = 0;
        }
        else {
            xError = currentX - targetX;
        }

        while (opModeIsActive() && (Math.abs(headingError) > 3) && (Math.abs(headingError) > 3) ) {
            // Read the current list

            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe   = Range.clip(xError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE) ;
            telemetry.addData("Program", "colourDrive");
            telemetry.addData("Target X",  " at %4d", targetX);
            telemetry.addData("Target heading",  " at %4.0f", targetHeading);
            telemetry.addData("Current X",  " at %4d", currentX);
            telemetry.addData("Current heading",  " at %4.0f", currentHeading);
            telemetry.update();
            // Apply desired axes motions to the drivetrain.
            if (currentX > 0) {
                moveRobot(0, strafe, -turn);  // we only want to rotate to keep heading and stafe to line up
                sleep(10);  // apply power for a little bit, then get new values.
            }
            orientation     = imu.getRobotYawPitchRollAngles();
            currentHeading  = orientation.getYaw(AngleUnit.DEGREES);
            headingError    = currentHeading - targetHeading;
        }
    }

    public void aprilTagDrive() {
                    //if(!inPosition) {   // DECODE add IF to test if we're in position to shoit
            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
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
                telemetry.addData("\n>","STEP 1 drive to tag 20\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (/* DECODE - remove gamepad test:  gamepad1.left_bumper && */ targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                if (Math.abs(rangeError) < 5) { // DECODE test if we are "close enough"
                    inPosition = true;          // DECODE set flag so we stop and launch ball
                }
                double  headingError    = desiredTag.ftcPose.bearing - DESIRED_BEARING;
                double  yawError        = desiredTag.ftcPose.yaw  - DESIRED_YAW;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }/* else {  // DECODE: remove the teleop code
                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }*/

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
            
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
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
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        /* DECODE - we're adding a second camera so init visionPortal later
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
        */
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
