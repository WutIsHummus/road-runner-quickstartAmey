package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.helpers.VisionHelper;
import org.firstinspires.ftc.teamcode.helpers.Helpers;
import org.firstinspires.ftc.teamcode.helpers.KalmanFilter;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * Experimental extension of MecanumDrive that uses AprilTags for relocalization.
 *
 * NOTE: I PLAN TO CREATE A CLEANED UP VERSION OF THIS TO DISTRIBUTE, this has some weird specific-to-my-codebase stuff
 * also, TODO: get rid of the kalman filter, vision helper, and multi cam
 *
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the BSD 3-Clause Clear License by Michael from 14343 and by Ryan Brott
 */
public class AprilTagDrive extends MecanumDrive {
    @Config
    static class Params {
        // distance FROM robot center TO camera (inches)
        // TODO: tune
        static Vector2d camera1Offset = new Vector2d(
                -6,
                4);
        // if you don't have a second camera this doesn't matter
        static Vector2d camera2Offset = new Vector2d(
                0,
                -5);//6);
        static double cameraYawOffset = Math.toRadians(180); // TODO: tune
        /*
         * Q model covariance (trust in model), default 0.1
         * R sensor covariance (trust in sensor), default 0.4
         */
        static double kalmanFilterQ = 0.1;
        static double kalmanFilterR = 0.4;
    }

    Vector2d cameraOffset;
    static final Params PARAMS = new Params();
    final AprilTagProcessor aprilTagBack;
    AprilTagProcessor aprilTagFront = null;
    public List<AprilTagDetection> totalDetections;
    public AprilTagDetection lastDetection;
    final KalmanFilter.Vector2dKalmanFilter posFilter;
    Pose2d aprilPose;
    Pose2d localizerPose;
    Vector2d filteredVector;
    boolean shouldTagCorrect = false;
    public VisionHelper vHelper = null;
    boolean frontCamActive = true;
    boolean backCamActive = true;
    /**
     * Init with just one camera; use instead of MecanumDrive
     * @param hardwareMap the hardware map
     * @param pose the starting pose
     * @param aprilTagBack your camera's AprilTagProcessor
     */
    public AprilTagDrive(HardwareMap hardwareMap, Pose2d pose, AprilTagProcessor aprilTagBack) {
        super(hardwareMap, pose);
        this.aprilTagBack = aprilTagBack;
        this.posFilter = new KalmanFilter.Vector2dKalmanFilter(PARAMS.kalmanFilterQ, PARAMS.kalmanFilterR);
        this.cameraOffset = Params.camera1Offset;

    }
    /**
     * Init with two cameras; use instead of MecanumDrive
     * @param hardwareMap the hardware map
     * @param pose the starting pose
     * @param aprilTagBack back camera's apriltag processor
     * @param aprilTagFront your second camera's AprilTagProcessor
     */
    public AprilTagDrive(HardwareMap hardwareMap, Pose2d pose, AprilTagProcessor aprilTagBack, AprilTagProcessor aprilTagFront) {
        super(hardwareMap, pose);
        this.aprilTagBack = aprilTagBack;
        this.aprilTagFront = aprilTagFront;
        this.posFilter = new KalmanFilter.Vector2dKalmanFilter(PARAMS.kalmanFilterQ, PARAMS.kalmanFilterR);
    }
    // VisionHelper is my helper class for switching between multiple cameras due to USB hub limitations
    // you can rip out all references to it if you want, you probably don't need it
    public AprilTagDrive(HardwareMap hardwareMap, Pose2d pose, VisionHelper vHelper) {
        super(hardwareMap, pose);
        this.aprilTagBack = vHelper.aprilTagBack;
        this.aprilTagFront = vHelper.aprilTagFront;
        this.posFilter = new KalmanFilter.Vector2dKalmanFilter(PARAMS.kalmanFilterQ, PARAMS.kalmanFilterR);
        this.vHelper = vHelper;
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        // RR standard: get the movement between loops from the localizer
        // RR assumes there's no way to get absolute position and gets relative between loops
        // note that this adds on top of existing pose, even if that pose was just corrected by apriltag
        Twist2dDual<Time> twist = localizer.update();
        localizerPose = pose.plus(twist.value());
        // Get the absolute position from the camera
        Vector2d aprilVector = getVectorBasedOnTags();


        // it's possible we can't see any tags, so we need to check for null
        if (aprilVector != null) {
            // if we can see tags, we use the apriltag position
            // however apriltags don't have accurate headings so we use the localizer heading
            // localizer heading, for us and in TwoDeadWheelLocalizer, is IMU and absolute-ish
            // TODO: apriltags unreliable at higher speeds? speed limit? global shutter cam? https://discord.com/channels/225450307654647808/225451520911605765/1164034719369941023

            // we input the change from odometry with the april absolute pose into the kalman filter
            filteredVector = posFilter.update(twist.value(), aprilVector);
            // then we add the kalman filtered position to the localizer heading as a pose
            pose = new Pose2d(aprilVector, localizerPose.heading); // TODO: aprilVector should be filteredVector to use kalman filter (kalman filter is untested)
            shouldTagCorrect = false; // TODO disable
        } else {
            // if we can't see tags, we use the localizer position to update the kalman fiter
            // not sure if this is logical at all?? seems to work
            filteredVector = posFilter.update(twist.value(), localizerPose.position);

            // then just use the existing pose
            pose = localizerPose;
        }



        // rr standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        return twist.velocity().value(); // trust the existing localizer for speeds; because I don't know how to do it with apriltags
    }
    public Vector2d getVectorBasedOnTags() {
        if (vHelper.frontCamActive) {
            frontCamActive = true;
            backCamActive = false;
        } else {
            backCamActive = true;
            frontCamActive = false;
        }
        List<AprilTagDetection> currentDetections = new ArrayList<>();
        if (backCamActive) {
            currentDetections = aprilTagBack.getDetections();
        }
        List<AprilTagDetection> cam2Detections = new ArrayList<>();
        if (aprilTagFront != null && frontCamActive) {
            cam2Detections = aprilTagFront.getDetections();
        }
        totalDetections = new ArrayList<>();
        totalDetections.addAll(currentDetections);
        totalDetections.addAll(cam2Detections);
        int realDetections = 0;
        Vector2d averagePos = new Vector2d(0,0); // starting pose to add the rest to
        if (totalDetections.isEmpty()) return null; // if we don't see any tags, give up (USES NEED TO HANDLE NULL)
        Vector2d RobotPos;

        // Step through the list of detections and calculate the robot position from each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) { // && !detection.metadata.name.contains("Small")) { // TODO: Change if we want to use wall tags?
                Vector2d tagPos = Helpers.toVector2d(detection.metadata.fieldPosition); // SDK builtin tag position
                double tagHeading = Helpers.quarternionToHeading(detection.metadata.fieldOrientation); // SDK builtin tag heading

                //RobotPos = calculateRobotPosFromTag(tagPos, tagHeading,localizerPose.heading.log(), detection); // calculate the robot position from the tag position
                RobotPos = getFCPosition(detection, localizerPose.heading.log(),false);

                // we're going to get the average here by adding them all up and dividingA the number of detections
                // we do this because the backdrop has 3 tags, so we get 3 positions
                // hopefully by averaging them we can get a more accurate position
                lastDetection = detection;
                averagePos = averagePos.plus(RobotPos);
                realDetections++;

            }
        }   // end for() loop

        if (!cam2Detections.isEmpty()) {
            // Step through the list of detections and calculate the robot position from each one.
            for (AprilTagDetection detection : cam2Detections) {
                if (detection.metadata != null) {  //&& !detection.metadata.name.contains("Small")) { // TODO: Change if we want to use wall tags?
                    Vector2d tagPos = Helpers.toVector2d(detection.metadata.fieldPosition); // SDK builtin tag position
                    double tagHeading = Helpers.quarternionToHeading(detection.metadata.fieldOrientation); // SDK builtin tag heading

                    //RobotPos = calculateRobotPosFromTag(tagPos, tagHeading,localizerPose.heading.log(), detection); // calculate the robot position from the tag position
                    RobotPos = getFCPosition(detection, localizerPose.heading.log(), true);

                    // we're going to get the average here by adding them all up and dividingA the number of detections
                    // we do this because the backdrop has 3 tags, so we get 3 positions
                    // hopefully by averaging them we can get a more accurate position
                    lastDetection = detection;
                    averagePos = averagePos.plus(RobotPos);
                    realDetections++;

                }
            }   // end for() loop
        }
        // divide by the number of detections to get the true average, as explained earlier
        return averagePos.div(realDetections);
    }

    // this is my original rotator code
    // it doesn't work lol
    @NonNull
    private static Vector2d calculateRobotPosFromTag(Vector2d tagPos, double tagHeading, double imuHeading, AprilTagDetection detection) {
        // TODO: I don't actually know trig, this is probably terrible
        double xPos;
        double yPos;
        //if (Math.abs(Math.toDegrees((imuHeading - PARAMS.cameraYawOffset) - tagHeading)) - 5 > 0) { // if the robot isn't within half a degree of straight up
        double tagRelHeading = imuHeading - PARAMS.cameraYawOffset + Math.toRadians(detection.ftcPose.bearing) - tagHeading;
        Vector2d camGlobalOffset = new Vector2d(
                PARAMS.camera1Offset.x * Math.cos(-imuHeading) - PARAMS.camera1Offset.y * Math.sin(-imuHeading),
                PARAMS.camera1Offset.x * Math.sin(-imuHeading) + PARAMS.camera1Offset.y * Math.cos(-imuHeading));
        xPos = tagPos.x - (Math.cos(tagRelHeading) * detection.ftcPose.y) - camGlobalOffset.x;
        yPos = tagPos.y - (Math.sin(tagRelHeading) * detection.ftcPose.y) - camGlobalOffset.y;
        /*} else {
            xPos = (tagPos.x - detection.ftcPose.y) - PARAMS.cameraOffset.x; // TODO; this will ONLY work for the backdrop tags
            yPos = (tagPos.y - detection.ftcPose.x) - PARAMS.cameraOffset.y;
        }*/

        return new Vector2d(xPos, yPos);
    }

    // getFCPosition credit Michael from team 14343 (@overkil on Discord)
    /**
     * @param botheading In Radians.
     * @return FC Pose of bot.
     */
    public Vector2d getFCPosition(AprilTagDetection detection, double botheading, boolean usingCam2) {
        Vector2d cameraOffset;
        if (usingCam2) {cameraOffset = Params.camera2Offset;} else {cameraOffset = Params.camera1Offset;}
        // get coordinates of the robot in RC coordinates
        // ensure offsets are RC
        double x = detection.ftcPose.x-cameraOffset.x;
        double y = detection.ftcPose.y-cameraOffset.y;

        // invert heading to correct properly
        botheading = -botheading;


        // rotate RC coordinates to be field-centric
        double x2 = x*Math.cos(botheading)+y*Math.sin(botheading);
        double y2 = x*-Math.sin(botheading)+y*Math.cos(botheading);
        double absX;
        double absY;
        // add FC coordinates to apriltag position
        // tags is just the CS apriltag library
        VectorF tagpose = getCenterStageTagLibrary().lookupTag(detection.id).fieldPosition;
        if (detection.metadata.id <= 6) {
            absX = tagpose.get(0) + y2;
            absY = tagpose.get(1) - x2;

        } else {
            absX = tagpose.get(0) - y2;
            absY = tagpose.get(1) + x2; // prev -

        }
        return new Vector2d(absX, absY);
    }


    public void correctWithTag() {
        shouldTagCorrect = true;
    }
    public Action CorrectWithTagAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                correctWithTag();
                return false;
            }
        };
    }
    // this position library credit Michael from team 14343 (@overkil on Discord)
    public static AprilTagLibrary getCenterStageTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(61.75f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(61.75f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(61.75f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(61.75f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(61.75f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(61.75f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }
}