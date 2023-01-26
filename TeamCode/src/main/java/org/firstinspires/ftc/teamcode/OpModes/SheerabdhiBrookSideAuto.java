package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.Utils.Team;


import java.util.HashMap;

@Autonomous
public class SheerabdhiBrookSideAuto extends BaseAuto {
    private enum ParkingPosition{
        LEFT,
        RIGHT,
        CENTER
    }
    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        int parkingPosition = 1;

        Pose2d startPose = new Pose2d(-36, 66.5,Math.toRadians(-90));
        Pose2d exampleEnd = new Pose2d(0, 0,Math.toRadians(0));
        Vector2d goToPole = new Vector2d(-36, 12.5);
//		Pose2d goToPole1 = new Pose2d(-35, 8, Math.toRadians(-90));
        Vector2d rotateFaceMedium = new Vector2d(-32.5, 14.5);
//		Vector2d rotateFaceMedium = new Vector2d(-32.5, 14.5);
//		Pose2d rotateFaceMedium1 = new Pose2d(-36, 6, Math.toRadians(50));
        Pose2d goToPark1 = new Pose2d(-12.5, 12.5, Math.toRadians(180));
//		Vector2d goToPark1 = new Vector2d(-12.5, 12.5);
        Pose2d goToPark3 = new Pose2d(-58, 12.5, Math.toRadians(45));
        Pose2d goToPark2 = new Pose2d(-35, 12.5, Math.toRadians(0));
//		Vector2d goToPark3 = new Vector2d(-58, 9);
//		Vector2d goToPark2 = new Vector2d(-35, 12.5);

        HashMap<Integer, Pose2d> parking = new HashMap<Integer, Pose2d>();
//		HashMap<Integer, Pose2d> toScoring = new HashMap<Integer, Pose2d>();
//		HashMap<Integer, Pose2d> rotation = new HashMap<Integer, Pose2d>();

        parking.put(1, goToPark1);
        parking.put(2, goToPark3);
        parking.put(3, goToPark2);

        Trajectory scoring = robot.drivetrain.getBuilder().trajectoryBuilder(startPose,false)
                .splineToConstantHeading(goToPole,Math.toRadians(270))
                .splineTo(rotateFaceMedium, Math.toRadians(45))
//                .lineToLinearHeading(parking.get(parkingPosition))
                .build();

        Trajectory parkTraj = robot.drivetrain.getBuilder().trajectoryBuilder(scoring.end(),false)
                .splineToLinearHeading(goToPark1,Math.toRadians(0))
                .build();

        Command auto = followRR(scoring).addNext(followRR(parkTraj));

        return auto;


    }

    @Override
    public Team getTeam() {
        return Team.BLUE;
    }
//    public void addCycle(Command command, ScoringCommandGroups commandGroups) {
//        command.addNext(multiCommand(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION),
//                        commandGroups.moveToIntakingRightAuto(),
//                        commandGroups.moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND)))
//                .addNext(commandGroups.moveHorizontalExtension(HorizontalExtension.mostlyAutoExtension))
//                .addNext(commandGroups.collectConeAuto());
//    }
}
