package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.TEAM;
import org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter;

@TeleOp(name=".pippersBLUE \uD83D\uDD35")
public class BLUE extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TeleopsStarter opmode = new TeleopsStarter(hardwareMap, TEAM.BLUE);

        waitForStart();
        RobotInitializers.enable();
        while (opModeIsActive()){
            opmode.update();
            RobotInitializers.telemetry.addData("Lift elevator", Lift.getPosition());
            RobotInitializers.telemetry.update();
        }
    }
}