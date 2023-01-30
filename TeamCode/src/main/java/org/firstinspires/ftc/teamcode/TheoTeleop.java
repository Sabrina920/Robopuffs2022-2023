package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOpppp", group="Iterative Opmode")


public class TheoTeleop extends OpMode{
    public RobotHardware rh = new RobotHardware (hardwareMap);
    @Override
    public void init() {
        rh.initialize();
    }
    @Override
    public void loop () {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.0; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        rh.drive(x, y, rx);
    }
}
