package org.firstinspires.ftc.teamcode.Autonomous.RRTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class LineToLinear180 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime time = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        double startTime = time.time();
        System.out.println(startTime);



        double endTime = time.time();
        System.out.println(endTime);
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.addData("Elapsed Time: ", endTime - startTime);
        telemetry.update();

        System.out.println(endTime - startTime);

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
