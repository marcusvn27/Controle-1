// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  Joystick joy = new Joystick(0);
  VictorSPX motorRightF = new VictorSPX(1);
  VictorSPX motorRightB = new VictorSPX(2);
  VictorSPX motorLeftF = new VictorSPX(3);
  VictorSPX motorLeftB = new VictorSPX(4);
  boolean bA, bB, bX, bY;
  double eixo1y, eixo1x, eixo2y, eixo2x, motoresdireita1, motoresesquerda1, motoresdireita2, motoresesquerda2,
      velomotorR, velomotorL, magnitude1, magnitude2, seno1, seno2;
  int pov;
  double buttonV = 1;

  @Override
  public void robotInit() {
    motorRightB.follow(motorRightF);
    motorLeftB.follow(motorLeftF);
    motorRightF.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    eixo1y = -joy.getRawAxis(1);
    eixo1x = joy.getRawAxis(0);
    eixo2y = -joy.getRawAxis(5);
    eixo2x = joy.getRawAxis(4);
    bA = joy.getRawButton(1);
    bB = joy.getRawButton(2);
    bX = joy.getRawButton(3);
    bY = joy.getRawButton(4);
    pov = joy.getPOV();

    magnitude1 = Math.sqrt(Math.pow(eixo1y, 2) + Math.pow(eixo1x, 2));
    magnitude2 = Math.sqrt(Math.pow(eixo2y, 2) + Math.pow(eixo2x, 2));
    seno1 = eixo1y / magnitude1;
    seno2 = eixo2y / magnitude2;

    // Botoes de controle de velocidade

    if (bX) // Força Máxima
      buttonV = 1;

    else if (bA) // Força Média
      buttonV = 0.5;

    else if (bB) // Força Mínima
      buttonV = 0.25;

    // Analogico Comum

    // Quadrante 1
    if (eixo1y >= 0 && eixo1x >= 0) {
      motoresesquerda1 = magnitude1*buttonV;
      motoresdireita1 = ((2 * seno1 - 1) * magnitude1)*buttonV;
      // Quadrante 2
    } else if (eixo1y >= 0 && eixo1x <= 0) {
      motoresesquerda1 = ((2 * seno1 - 1) * magnitude1)*buttonV;
      motoresdireita1 = magnitude1*buttonV;
      // Quadrante 3
    } else if (eixo1y < 0 && eixo1x < 0) {
      motoresesquerda1 = -magnitude1*buttonV;
      motoresdireita1 = ((2 * seno1 + 1) * magnitude1)*buttonV;
      // Quadrante 4
    } else if (eixo1y < 0 && eixo1x >= 0) {
      motoresesquerda1 = ((2 * seno1 + 1) * magnitude1)*buttonV;
      motoresdireita1 = -magnitude1*buttonV;
    }

    // Analogico Reverso

    // Quadrante 1
    if (eixo2y >= 0 && eixo2x >= 0) {
      motoresdireita2 = -magnitude2*buttonV;
      motoresesquerda2 = ((-2 * seno2 + 1) * magnitude2)*buttonV;
      // Quadrante 2
    } else if (eixo2y >= 0 && eixo2x < 0) {
      motoresdireita2 = ((-2 * seno2 + 1) * magnitude2)*buttonV;
      motoresesquerda2 = -magnitude2*buttonV;
      // Quadrante 3
    } else if (eixo2y < 0 && eixo2x < 0) {
      motoresdireita2 = magnitude2*buttonV;
      motoresesquerda2 = ((-2 * seno2 - 1) * magnitude2)*buttonV;
      // Quadrante 4
    } else if (eixo2y < 0 && eixo2x >= 0) {
      motoresdireita2 = ((-2 * seno2 - 1) * magnitude2)*buttonV;
      motoresesquerda2 = magnitude2*buttonV;
    }

    

    if (Math.abs(eixo1x) < 0.04 && Math.abs(eixo1y) < 0.04) {
      motoresdireita1 = 0;
      motoresesquerda1 = 0;
    }
    if (Math.abs(eixo2x) < 0.04 && Math.abs(eixo2y) < 0.04) {
      motoresdireita2 = 0;
      motoresesquerda2 = 0;
    }

    switch (pov) {
      case 0:
        velomotorL = 0.25;
        velomotorR = 0.25;
        break;
      case 45:
        velomotorL = 0.25;
        velomotorR = 0;
        break;
      case 90:
        velomotorL = 0.25;
        velomotorR = -0.25;
        break;
      case 135:
        velomotorL = 0;
        velomotorR = -0.25;
        break;
      case 180:
        velomotorL = -0.25;
        velomotorR = -0.25;
        break;
      case 225:
        velomotorL = -0.25;
        velomotorR = 0;
        break;
      case 270:
        velomotorL = -0.25;
        velomotorR = 0.25;
        break;
      case 315:
        velomotorL = 0;
        velomotorR = 0.25;
        break;
    }

    if (Math.abs(motoresdireita2) > 0.04 && Math.abs(motoresdireita1) > 0.04) {
      velomotorL = 0;
      velomotorR = 0;
    } else if (Math.abs(motoresdireita1) > 0.04 && Math.abs(motoresdireita2) < 0.04) {
      velomotorL = motoresesquerda1;
      velomotorR = motoresdireita1;
    } else if (Math.abs(motoresdireita1) < 0.04 && Math.abs(motoresdireita2) > 0.04) {
      velomotorL = motoresesquerda2;
      velomotorR = motoresdireita2;
    } else {
      velomotorL = 0;
      velomotorR = 0;
    }

    SmartDashboard.putNumber("motores esquerda1", motoresesquerda1);
    SmartDashboard.putNumber("motores direita1", motoresdireita1);
    SmartDashboard.putNumber("motores esquerda2", motoresesquerda2);
    SmartDashboard.putNumber("motores direita2", motoresdireita2);
    SmartDashboard.putNumber("velocidade final D", velomotorR);
    SmartDashboard.putNumber("velocidade final E", velomotorL);

  }
}
