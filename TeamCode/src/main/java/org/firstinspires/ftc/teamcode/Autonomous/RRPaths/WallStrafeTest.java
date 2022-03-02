package org.firstinspires.ftc.teamcode.Autonomous.RRPaths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.BarCodeDuckPipeline;
import org.firstinspires.ftc.teamcode.threadedhardware.HardwareThread;
import org.firstinspires.ftc.teamcode.threadedhardware.RoadRunnerConfiguration;
import org.firstinspires.ftc.teamcode.threadedhardware.Sequence;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class WallStrafeTest extends LinearOpMode {

    public static double DISTANCE = 50, scoreX = -76, intakeY = 0, intakeAngleOffset = Math.toRadians(8), intakeAngle = 0;

    public Pose2d intakePose = new Pose2d(-16, intakeY);
    public Pose2d spline2 = new Pose2d(-42, 0);
    public Pose2d spline1 = new Pose2d(-48, 0);
    public Pose2d scorePose = new Pose2d(scoreX, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        RoadRunnerConfiguration drive = new RoadRunnerConfiguration(hardwareMap);
        HardwareThread hardware = new HardwareThread(hardwareMap, drive);
        hardware.start();

        sleep(500);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        Sequence adjustPose = new Sequence(() -> {
            boolean pressed = false, intakePress = false;
            while(!isStopRequested() && opModeIsActive()) {
                HardwareThread.waitForCycle();

                if(gamepad1.dpad_left && !pressed) {
                    scoreX += 1;
                    pressed = true;
                }
                else if(gamepad1.dpad_right && !pressed) {
                    scoreX -= 1;
                    pressed = true;
                }
                else if(!gamepad1.dpad_left && !gamepad1.dpad_right) {
                    pressed = false;
                }

                if(gamepad1.dpad_up && !intakePress) {
                    intakeY -= 3;
                    intakeAngle -= intakeAngleOffset;
                    System.out.println("Here 1");
                    intakePress = true;
                }
                else if(gamepad1.dpad_down && !intakePress && intakeY < 0) {
                    intakeY += 3;
                    intakeAngle += intakeAngleOffset;
                    intakePress = true;
                }
                else if(!gamepad1.dpad_up && !gamepad1.dpad_down) {
                    intakePress = false;
                }
            }
        });
        Thread adjustThread = new Thread(adjustPose);

        waitForStart();

        adjustThread.start();

        while (opModeIsActive() && !isStopRequested()) {

            Pose2d pose = drive.getPoseEstimate();

            ElapsedTime time = new ElapsedTime();

            double cos = Math.abs(Math.cos(drive.imu.get()[0]));
            double sin = Math.abs(Math.sin(drive.imu.get()[0]));
            double front = drive.front.get()[0] * cos;
            double left = drive.left.get()[0] * cos - 8 * sin;

            while(time.seconds() < 1 && (front > 32 || left > Math.abs(intakeY) + 10)) {
                HardwareThread.waitForCycle();
                front = drive.front.get()[0] * cos;
                left = drive.left.get()[0] * cos - 16 * sin;
            }
            drive.setPoseEstimate(new Pose2d(time.seconds() >= 0.5 ? pose.getX() : 2 - front, time.seconds() >= 0.5 ? pose.getY() : 2 - left)); //Need to adjust

            scorePose = new Pose2d(scoreX, 0);

            System.out.println("Left: " + left + ", front: " + front);
            System.out.println("Pose: " + drive.getPoseEstimate());

            telemetry.addLine("ScoreX: " + scoreX + ", IntakeY: " + intakeY);
            telemetry.addLine("Pose: " + drive.getPoseEstimate());
            telemetry.addLine("Left: " + left + ", front: " + front);
            telemetry.update();

            Trajectory trajectoryForward = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                    .splineToSplineHeading(spline2, Math.toRadians(180))
                    .splineToSplineHeading(scorePose, Math.toRadians(180))
                    .build();
            drive.followTrajectory(trajectoryForward);

            pose = drive.getPoseEstimate();

            time = new ElapsedTime();

            cos = Math.abs(Math.cos(drive.imu.get()[0]));

            while(time.seconds() < 0.5 && left > 12) {
                HardwareThread.waitForCycle();
                left = drive.left.get()[0] * cos;
            }
            drive.setPoseEstimate(new Pose2d(pose.getX(), time.seconds() >= 0.5 ? pose.getY() : 2 - left));

            intakePose = new Pose2d(-16, intakeY);

            Trajectory trajectoryBackward = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineToSplineHeading(spline1, 0)
                    .splineTo(intakePose.vec(), intakeAngle)
                    .build();
            drive.followTrajectory(trajectoryBackward);
        }

        hardware.Stop();
    }
}