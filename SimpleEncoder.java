package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonmous(name = "Encoder", group = "")
@Disabled
class Encoder extends LinearOpMode {
  // private DcMotor;
  private DcMotor bottomright;
  private DcMotor topright;
  private DcMotor topleft;
  private DcMotor bottomleft;
  private DcMotor pulley;
  private Servo trayright;
  private Servo trayleft;

  static final double COUNTS_PER_MOTOR_REV = 383.6;
  static final double DRIVE_SPEED = 0.6;
  static final double TURN_SPEED = 0.5;

  public void resetMotors() {
    bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  public void RunEncoders() {

    topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

  }

  public void DriveForward(double power, int TLdistance, int TRdistance, int BLdistance, int BRdistance) {
    resetMotors();

    topleft.setTargetPosition(TLdistance);
    topright.setTargetPosition(TRdistance);
    bottomleft.setTargetPosition(BLdistance);
    bottomright.setTargetPosition(BRdistance);

    // Turn On RUN_TO_POSITION
    topleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    topright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bottomleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bottomright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    RunEncoders();

    topleft.setPower(TLdistance);
    topright.setPower(TRdistance);
    bottomleft.setPower(BLdistance);
    bottomright.setPower(BRdistance);

    if (opModeIsActive()) {

      while (opModeIsActive()
          && (topleft.isBusy() && topright.isBusy() && bottomleft.isBusy() && bottomright.isBusy())) {

        telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d", topleft.getCurrentPosition(),
            topright.getCurrentPosition(), bottomleft.getCurrentPosition(), bottomright.getCurrentPosition());
        telemetry.update();

      }
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

  }

  @Override
  public void runOpMode() {
    trayright = hardwareMap.servo.get("trayright");
    trayleft = hardwareMap.servo.get("trayleft");
    // motors
    pulley = hardwareMap.dcMotor.get("pulley");
    bottomright = hardwareMap.dcMotor.get("bottomright");
    topright = hardwareMap.dcMotor.get("topright");
    topleft = hardwareMap.dcMotor.get("topleft");
    bottomleft = hardwareMap.dcMotor.get("bottomleft");

    topleft.setDirection(DcMotor.Direction.REVERSE);
    bottomleft.setDirection(DcMotor.Direction.REVERSE);

    // reset the motors
    resetMotors();
    // Turn encoder mode on
    RunEncoders();

    waitForStart();

    DriveForward(DRIVE_SPEED, COUNTS_PER_MOTOR_REV * 1, COUNTS_PER_MOTOR_REV * 1, COUNTS_PER_MOTOR_REV * 1,
        COUNTS_PER_MOTOR_REV * 1);
  }

}