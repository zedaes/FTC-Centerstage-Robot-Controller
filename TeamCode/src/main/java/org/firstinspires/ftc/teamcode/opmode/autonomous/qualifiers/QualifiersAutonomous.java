package org.firstinspires.ftc.teamcode.opmode.autonomous.qualifiers;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.opmode.autonomous.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.opmode.autonomous.vision.apriltag.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.teamcode.opmode.autonomous.vision.propdetection.TeamPropDetectionPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class QualifiersAutonomous extends LinearOpMode {
    public ElapsedTime stopwatch;
    double startX = 0;
    double startY = 0;
    Vector2d startVector = new Vector2d(startX,startY);
    double startHeading = Math.toRadians(0);
    int position = 3;
    Pose2d startPose = new Pose2d(startVector, startHeading);
    Pose2d currentPose = new Pose2d(startVector, startHeading);

    // define camera
    OpenCvCamera camera;
    // prop detection
    TeamPropDetectionPipeline teamPropDetectionPipeline;

    // apriltag stuff
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    int numFramesWithoutDetection = 0;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    List<Integer> tags = Arrays.asList(0, 0, 0);
    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        teamPropDetectionPipeline = new TeamPropDetectionPipeline();

        MecanumDrive robot = new MecanumDrive(hardwareMap, startPose);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();
        stopwatch = new ElapsedTime();
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        currentPose = robot.pose;
        switch (teamPropDetectionPipeline.teamPropZone){
            case 1:
                telemetry.addLine("left");
                goToPixelPosition1(robot, currentPose);
                break;
            case 2:
                telemetry.addLine("middle");
                goToPixelPosition2(robot, currentPose);
                break;
            case 3:
                telemetry.addLine("right");
                goToPixelPosition3(robot, currentPose);
                break;
        }
        camera.setPipeline(aprilTagDetectionPipeline);



    }

    public void goToPixelPosition1(MecanumDrive robot, Pose2d pose){
        Actions.runBlocking(
                robot.actionBuilder(pose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
    }
    public void goToPixelPosition2(MecanumDrive robot, Pose2d pose){
        Actions.runBlocking(
                robot.actionBuilder(pose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
    }
    public void goToPixelPosition3(MecanumDrive robot, Pose2d pose){
        Actions.runBlocking(
                robot.actionBuilder(pose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
    }

    public ArrayList getAprilTagDetections(ElapsedTime stopwatch, long searchTime) {
        ArrayList tagsDetected = null;
        while (stopwatch.time() < stopwatch.time() + searchTime) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            tagsDetected = null;

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tags.contains(tag) || !tagsDetected.contains(tag)) {
                        tagsDetected.add(tag);
                        aprilTagToTelemetry(tag);
                    }
                }
            }
        }
        return (tagsDetected);
    }

    public void aprilTagToTelemetry(AprilTagDetection detection) {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }

    public int getAprilTagID(AprilTagDetection detection){
        return detection.id;
    }

    public long getAprilTagTranslationX(AprilTagDetection detection){
        return (long) (detection.pose.x*FEET_PER_METER);
    }

    public long getAprilTagTranslationY(AprilTagDetection detection){
        return (long) (detection.pose.y*FEET_PER_METER);
    }

    public long getAprilTagTranslationZ(AprilTagDetection detection){
        return (long) (detection.pose.z*FEET_PER_METER);
    }

    public float getAprilRotationYaw(AprilTagDetection detection){
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        return rot.firstAngle;
    }

    public float getAprilRotationPitch(AprilTagDetection detection){
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        return rot.secondAngle;
    }

    public float getAprilRotationRoll(AprilTagDetection detection){
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        return rot.thirdAngle;
    }



}