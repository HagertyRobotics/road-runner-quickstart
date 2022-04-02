package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTricycleDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleTricycleDrive drive = new SampleTricycleDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,0));
        Trajectory forward = drive.trajectoryBuilder(new Pose2d(0,0,0), false)
                .splineTo(new Vector2d(30, 30), 0)
                .splineTo(new Vector2d(70,0),0)
                .forward(10)
                .build();
        Trajectory backward = drive.trajectoryBuilder(forward.end())
                .back(65)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            drive.setPoseEstimate(new Pose2d());
            drive.followTrajectory(forward);
            sleep(2000);
            drive.followTrajectory(backward);

        }

        //drive.followTrajectory();


//        Trajectory traj = drive.trajectoryBuilder(new Pose2d(z))
//                .splineTo(new Vector2d(30, 30), 0)
//                .build();
//
//        drive.followTrajectory(traj);
//
//        sleep(2000);
//
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
    }
}
