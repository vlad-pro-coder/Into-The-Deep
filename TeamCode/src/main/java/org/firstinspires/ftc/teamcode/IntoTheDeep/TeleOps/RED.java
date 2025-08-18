package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.TEAM;
import org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter;

@TeleOp(name = ".peppersRED \uD83D\uDFE5")
public class RED extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        TeleopsStarter opmode = new TeleopsStarter(hardwareMap, TEAM.RED,gamepad1,gamepad2);
        Lift.DoingAuto = false;
        Extendo.DoingAuto = false;

        waitForStart();
        RobotInitializers.enable();

        while (opModeIsActive()){
            opmode.update();
        }
    }
}