package org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.TuningOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOp.BlueQualTeleOp;

import java.util.Arrays;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class Test extends LinearOpMode {

    public MinVelocityConstraint slowVel = new MinVelocityConstraint(
            Arrays.asList(new AngularVelocityConstraint(Math.toRadians(360)), new MecanumVelocityConstraint(15,
                    DriveConstants.TRACK_WIDTH)));
    public ProfileAccelerationConstraint slowAccel = new ProfileAccelerationConstraint(10);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.leftIntakeLift.setTargetPosition(BlueQualTeleOp.intakeLiftLevels[0]);
        drive.leftIntakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftIntakeLift.setPower(0.4);

        drive.leftIntakeSpinner.setPower(-1);

        Trajectory pickup = drive.trajectoryBuilder(drive.getPoseEstimate(), true, slowVel, slowAccel)
                .back(40)
                .build();
        drive.followTrajectoryAsync(pickup);

        while (!isStopRequested() && drive.isBusy() && drive.leftLimit.getState()) {
            System.out.println("Limit: " + drive.leftLimit.getState());
            drive.update();
        }

        drive.setPoseEstimate(pickup.end());

        drive.update();

        drive.setMotorPowers(0, 0, 0, 0);
    }
}
