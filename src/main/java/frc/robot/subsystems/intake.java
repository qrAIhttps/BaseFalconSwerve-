package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intake extends SubsystemBase {
  // pid
  PIDController topPID;
  PIDController bottomPID;

  // switch
  private DigitalInput bottomSwitch;
  private DigitalInput topSwitch;
  // top intake motor declarations
  private final TalonFX m_topLeftIntake ;
  private final TalonFX m_topRightIntake;
  private final TalonFX m_topRollers;
  // bottom intake
  private final TalonFX m_bottomLeftIntake;
  private final TalonFX m_bottomRightIntake;
  private final TalonFX m_bottomRollers;

  public intake() {
    // pid
    topPID = new PIDController(Constants.MKINTAKE.kP, Constants.MKINTAKE.kI, Constants.MKINTAKE.kD,
        Constants.MKINTAKE.kF);
    bottomPID = new PIDController(Constants.MKINTAKE.kP, Constants.MKINTAKE.kI, Constants.MKINTAKE.kD,
        Constants.MKINTAKE.kF);

    // switch
    bottomSwitch = new DigitalInput(9);
    topSwitch = new DigitalInput(6);
    // top intake motor declarations
    m_topLeftIntake = new TalonFX(Constants.CANID.topLeftIntakeCANID);
    m_topRightIntake = new TalonFX(Constants.CANID.topRightIntakeCANID);
    m_topRollers = new TalonFX(Constants.CANID.topRollersCANID);
    // bottom intake
    m_bottomLeftIntake = new TalonFX(Constants.CANID.bottomLeftIntakeCANID);
    m_bottomRightIntake = new TalonFX(Constants.CANID.bottomRightIntakeCANID);
    m_bottomRollers = new TalonFX(Constants.CANID.bottomRollersCANID);

  }
  public void topIntakePID(double setpoint){
    m_topRightIntake.set(ControlMode.PercentOutput,topPID.calculate((m_topLeftIntake.getSelectedSensorPosition()+m_topRightIntake.getSelectedSensorPosition())/2, setpoint));
    m_topLeftIntake.set(ControlMode.PercentOutput,topPID.calculate((m_topLeftIntake.getSelectedSensorPosition()+m_topRightIntake.getSelectedSensorPosition())/2, setpoint));
  }

  public void bottomIntakePID(double setpoint){
    m_bottomRightIntake.set(ControlMode.PercentOutput,bottomPID.calculate((m_bottomLeftIntake.getSelectedSensorPosition()+m_bottomRightIntake.getSelectedSensorPosition())/2, setpoint));
    m_bottomLeftIntake.set(ControlMode.PercentOutput,bottomPID.calculate((m_bottomLeftIntake.getSelectedSensorPosition()+m_bottomRightIntake.getSelectedSensorPosition())/2, setpoint));
  }
  public CommandBase closeTop(){
    return this.runOnce(()->topIntakePID(0));
  }
  public CommandBase midTop(){
    return this.runOnce(()->topIntakePID(-36900));
  }
  public CommandBase bottomTop(){
    return this.runOnce(()->topIntakePID(-42000));
  }

  public CommandBase closeBottom(){
    return this.runOnce(()->topIntakePID(0));
  }
  public CommandBase midBottom(){
    return this.runOnce(()->topIntakePID(36900));
  }
  public CommandBase bottomBottom(){
    return this.runOnce(()->topIntakePID(42000));
  }
  // public CommandBase openTop(){
  // return this.runOnce(()->{
  // topLeftIntake.set(1);
  // });
  // }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    // builder.addBooleanProperty("extended", () -> m_hatchSolenoid.get() ==
    // kForward, null);
  }
}