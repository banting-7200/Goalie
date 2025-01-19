package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
  AddressableLEDBuffer ledInstanceBuffer;
  AddressableLED ledInstance;
  private Timer lightDelayTimer;
  private double currentTime;
  private double lastTime;
  private double lightBrightness;
  private double a;
  private double totalTime;
  private double increasingTime;
  private int rainbowFirstPixelHue = 0;
  private int currentEffect;
  private int lightState;
  private double red;
  private double green;
  private double blue;

  public LightsSubsystem(int lightsPort, int lightsLength) {
    ledInstance = new AddressableLED(lightsPort);
    ledInstanceBuffer = new AddressableLEDBuffer(lightsLength);
    ledInstance.setLength(lightsLength);
    ledInstance.start();
    // lightDelayTimer.start();
    currentEffect = 1;
  }

  public void run() {
    // currentTime = lightDelayTimer.getFPGATimestamp();
    ledInstance.setData(ledInstanceBuffer);
  }

  public void setLEDColorWithBrightness(
      int red, int green, int blue, double brightnessBetween0And1) {
    this.red = red * brightnessBetween0And1;
    this.green = green * brightnessBetween0And1;
    this.blue = blue * brightnessBetween0And1;
    int finalRed = Math.round((int) this.red);
    int finalGreen = Math.round((int) this.green);
    int finalBlue = Math.round((int) this.blue);
    // It works and I dont care
    if (finalRed > 255) {
      finalRed = 255;
    }
    if (finalRed < 0) {
      finalRed = 0;
    }
    if (finalGreen > 255) {
      finalGreen = 255;
    }
    if (finalGreen < 0) {
      finalGreen = 0;
    }
    if (finalBlue > 255) {
      finalBlue = 255;
    }
    if (finalBlue < 0) {
      finalBlue = 0;
    }

    for (int x = 0; x < ledInstanceBuffer.getLength(); x++) {
      ledInstanceBuffer.setRGB(x, finalRed, finalGreen, finalBlue);
    }
  }

  public void rainbow() {
    for (var i = 0; i < ledInstanceBuffer.getLength(); i++) {
      int hue = (rainbowFirstPixelHue + (i * 180 / ledInstanceBuffer.getLength())) % 180;
      ledInstanceBuffer.setHSV(i, hue, 255, 128);
    }

    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  public void blinkingFade(double riseTotalTime, double holdOnTotalTime, double fadeOutTotalTime) {

    // Blinking fade works by feeding the code a time for each section of the effect
    // A "rising" effect is the LEDS getting brighter, feeding a total time of 5 seconds, means the
    // lights will go from 0 to max in 5 secs
    // Hold on, is keeping the lights on
    // fade out is dimming the LEDS on 0

    // y= at^2
    switch (lightState) {
      case 1:
        // Rising brightness effect
        if (lightBrightness < 256) {
          if (currentTime - lastTime > 50) {
            lastTime = currentTime;
            increasingTime += 0.05;
            a = 255 / Math.sqrt(increasingTime - riseTotalTime);
            lightBrightness = a * (Math.sqrt(increasingTime));
            setLEDColorWithBrightness((int) red, (int) green, (int) blue, lightBrightness);
            lightDelayTimer.reset();
          }
        } else {
          lightState = 2;
        }
        break;

      case 2:
        // Keep on the lights on effect
        if (currentTime - lastTime > holdOnTotalTime) {
          lastTime = currentTime;
          lightState = 3;
          lightDelayTimer.reset();
        }
        break;
      case 3:
        // Fading out brightness effect
        if (currentTime - lastTime > 50) {
          lastTime = currentTime;
          increasingTime += 0.05;
          a = 255 / Math.sqrt(increasingTime - fadeOutTotalTime);
          lightBrightness = a * (Math.sqrt(increasingTime - totalTime));
          lightDelayTimer.reset();

        } else {
          lightState = 1;
        }
        break;
    }
  }

  public void turnLightsOff() {
    for (int i = 0; i < ledInstanceBuffer.getLength(); i++) {
      ledInstanceBuffer.setLED(i, Color.kBlack);
    }
  }
}
