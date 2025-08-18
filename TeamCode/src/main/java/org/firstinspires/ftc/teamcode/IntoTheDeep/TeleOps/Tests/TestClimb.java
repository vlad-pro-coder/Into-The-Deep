package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Climb.Climb.ClimbActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Climb.Climb.updateClimb;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Climb.ClimbConstants.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class TestClimb extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);
        RobotInitializers.InitializeForOperation();
        tasks.AddAnotherScheduler(ClimbActions());
        Extendo.DoingAuto = true;

        Outtake.closeClawTight();

        while(opModeInInit()){

            RobotInitializers.clearCache();

            Lift.update();
            Extendo.update();
            Outtake.update();

            RobotInitializers.Dashtelemetry.addData("pitch",RobotInitializers.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        }

        waitForStart();

        while (opModeIsActive()){
            RobotInitializers.clearCache();

            updateClimb();
        }
    }
}
