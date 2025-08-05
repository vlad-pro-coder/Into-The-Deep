package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class OverheadTest extends LinearOpMode {
    public static double pos=120,lastpos = 120;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            if(lastpos != pos) {
                Outtake.setOverHeadPos(pos);
                lastpos = pos;
            }

            Outtake.armProfile.update();
            Outtake.Overheads1.setAngle(Outtake.armProfile.getPosition());
            Outtake.Overheads2.setAngle(Outtake.armProfile.getPosition());


        }
    }
}
