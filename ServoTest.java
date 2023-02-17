package Provs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Jan 14")
public class ServoTest extends OpMode
{
    private Servo hClaw, vClaw, vPitch, wiper, align;
    double h = 0.92;//open 0.92 | close 0.76
    double vC = 0.5;//open 0.56 | close 0.521
    double vP = 0.7;//in 0.0 | out 0.673
    double w = 0.26;//in 0.26 | out 0.68
    double a = 0;// in 0.0 | out 0.28

    @Override
    public void init() {

        hClaw = hardwareMap.get(Servo.class, "hClaw");
        vClaw = hardwareMap.get(Servo.class, "vClaw");
        vPitch = hardwareMap.get(Servo.class, "vPitch");
        wiper = hardwareMap.get(Servo.class, "wiper");
        align = hardwareMap.get(Servo.class, "align");

    }

    @Override
    public void loop() {

        hClaw.setPosition(h);
        vClaw.setPosition(vC);
        vPitch.setPosition(vP);
        wiper.setPosition(w);
        align.setPosition(a);

        if (gamepad1.y && h < 1) {
            h += 0.001;
        }

        if (gamepad1.a && h > 0) {
            h -= 0.001;
        }

        if (gamepad1.b && vC < 1) {
            vC += 0.001;
        }

        if (gamepad1.x && vC > 0) {
            vC -= 0.001;
        }

        if (gamepad1.dpad_up && vP < 1) {
            vP += 0.001;
        }

        if (gamepad1.dpad_down && vP > 0) {
            vP -= 0.001;
        }

        if (gamepad1.dpad_right && w < 1) {
            w += 0.001;
        }

        if (gamepad1.dpad_left && w > 0) {
            w -= 0.001;
        }

        if (gamepad1.right_bumper && w < 1) {
            a += 0.001;
        }

        if (gamepad1.left_bumper && w > 0) {
            a -= 0.001;
        }

        telemetry.addData("hPitch", h);
        telemetry.addData("vClaw", vC);
        telemetry.addData("vPitch", vP);
        telemetry.addData("wiper", w);
        telemetry.addData("align", a);
        telemetry.update();
    }
}