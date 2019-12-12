package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonmous(name = "Encoder", group = "")
@Disabled
public class Encoder extends LinearOpMode {

  private ElapsedTime runtime = new ElapsedTime();

  // private DcMotor pulley;
  private DcMotor bottomright;
  private DcMotor topright;
  private DcMotor topleft;
  private DcMotor bottomleft;
  private DcMotor pulley;
  private Servo trayright;
  private Servo trayleft;
  // constants
  static final double COUNTS_PER_MOTOR_REV = 3892; // eg: Gobilda Motor Encoder
  static final double DRIVE_GEAR_REDUCTION = 2.0; // This is < 1.0 if geared UP
  static final double WHEEL_DIAMETER_INCHES = 3.93701; // For figuring circumference
  static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
      / (WHEEL_DIAMETER_INCHES * 3.1415);
  static final double DRIVE_SPEED = 0.6;
  static final double TURN_SPEED = 0.5;

  public void strafe(double tL, double tR, double bR, double bL) {
    topleft.setPower(tL);
    topright.setPower(tR);
    bottomleft.setPower(bL);
    bottomright.setPower(bR);
  }

  public void turn_90(double lMotor, double rMotor) {
    topleft.setPower(lMotor);
    bottomright.setPower(rMotor);
    topright.setPower(rMotor);
    bottomleft.setPower(lMotor);
    sleep(500);
    topleft.setPower(0);
    topright.setPower(0);
    bottomleft.setPower(0);
    bottomright.setPower(0);
  }

  public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
    int newTopLeftTarget;
    int newBottomLeftTarget;
    int newTopRightTarget;
    int newBottomRightTarget;

    // Ensure that the opmode is still active
    if (opModeIsActive()) {

      // Determine new target position, and pass to motor controller
      newTopLeftTarget = topleft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
      newBottomLeftTarget = bottomleft.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
      newTopRightTarget = topright.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
      newBottomRightTarget = bottomright.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

      topleft.setTargetPosition(newLeftTarget);
      bottomleft.setTargetPosition(newRightTarget);
      topright.setTargetPosition(newLeftTarget);
      bottomright.setTargetPosition(newRightTarget);

      // Turn On RUN_TO_POSITION
      topleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      topright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      bottomleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      bottomright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // reset the timeout time and start motion.
      runtime.reset();

      topleft.setPower(Math.abs(speed));
      topright.setPower(Math.abs(speed));
      bottomleft.setPower(Math.abs(speed));
      bottomright.setPower(Math.abs(speed));

      // keep looping while we are still active, and there is time left, and both
      // motors are running.
      // Note: We use (isBusy() && isBusy()) in the loop test, which means that when
      // EITHER motor hits
      // its target position, the motion will stop. This is "safer" in the event that
      // the robot will
      // always end the motion as soon as possible.
      // However, if you require that BOTH motors have finished their moves before the
      // robot continues
      // onto the next step, use (isBusy() || isBusy()) in the loop test.
      while (opModeIsActive() && (runtime.seconds() < timeoutS)
          && (topleft.isBusy() && topright.isBusy() && bottomleft.isBusy() && bottomright.isBusy())) {

        // Display it for the driver.
        telemetry.addData("Path1", "Running to %7d :%7d", newTopLeftTarget, newBottomRightTarget, newBottomLeftTarget,
            newTopRightTarget);

        telemetry.addData("Path2", "Running at %7d :%7d", topleft.getCurrentPosition(), topright.getCurrentPosition(),
            bottomleft.getCurrentPosition(), bottomright.getCurrentPosition());
        telemetry.update();
      }

      // Stop all motion;
      topleft.setPower(0);
      topright.setPower(0);
      bottomright.setPower(0);
      bottomleft.setPower(0);

      // Turn off RUN_TO_POSITION
      topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      // sleep(250); // optional pause after each move
    }
  }

  @Override
  public void runOpMode() {
    // servos
    trayright = hardwareMap.servo.get("trayright");
    trayleft = hardwareMap.servo.get("trayleft");
    // motors
    pulley = hardwareMap.dcMotor.get("pulley");
    bottomright = hardwareMap.dcMotor.get("bottomright");
    topright = hardwareMap.dcMotor.get("topright");
    topleft = hardwareMap.dcMotor.get("topleft");
    bottomleft = hardwareMap.dcMotor.get("bottomleft");

    // reset the motors
    bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Turn encoder mode on
    bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();

    // encoderDrive(double speed, double leftInches, double rightInches, double
    // timeoutS)

    // you will code :) 🧕🏻
    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    encoderDrive(DRIVE_SPEED, 10, 10, 5.0); // S1: Forward 47 Inches with 5 Sec timeout
    // encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn Right 12 Inches with 4
    // Sec timeout
    // encoderDrive(DRIVE_SPEED, -24, -24, 4.0); // S3: Reverse 24 Inches with 4 Sec
    // timeout

    // trayright.setPosition(1.0); // S4: Stop and close the claw.
    // trayleft.setPosition(0.0);
    // sleep(1000); // pause for servos to move

    telemetry.addData("Path", "Complete");
    telemetry.update();
  }
}