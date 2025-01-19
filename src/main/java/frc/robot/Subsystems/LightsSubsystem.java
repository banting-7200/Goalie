package frc.robot.Subsystems;

import java.lang.ModuleLayer.Controller;
import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LightsSubsystem {
    private AddressableLED ledInstance;
    private AddressableLEDBuffer bufferInstance;
    private int rainbowFirstPixelHue = 0;
    private long currentTime = System.currentTimeMillis(), previousTime = 0;
    boolean areLightsOn = false;

    public LightsSubsystem(int lightPort, int lightCount) {
        ledInstance = new AddressableLED(lightPort);
        bufferInstance = new AddressableLEDBuffer(lightCount);
        ledInstance.setLength(lightCount);
        ledInstance.start();
    }

    public void run() {
      currentTime = System.currentTimeMillis();
        ledInstance.setData(bufferInstance);
    }

   
    
    void blinkingSolidColor(int red, int green, int blue, double blinkRate) {
      

      if (currentTime - previousTime > blinkRate) {
        if(!areLightsOn){
          for (int x = 0; x < bufferInstance.getLength(); x++) {
            bufferInstance.setRGB(x, red, green, blue);
          }
        } else {
          for (int i = 0; i < bufferInstance.getLength(); i++) {
            bufferInstance.setLED(i, Color.kBlack);
          }
        }

        areLightsOn = !areLightsOn;
        previousTime = currentTime;
      }

    }

    void rainbow() {
      for (var i = 0; i < bufferInstance.getLength(); i++) {
        int hue = (rainbowFirstPixelHue + (i * 180 / bufferInstance.getLength())) % 180;
        bufferInstance.setHSV(i, hue, 255, 128);
      }

      rainbowFirstPixelHue += 3;
      rainbowFirstPixelHue %= 180;
    }
    




     void off() {
      for (int i = 0; i < bufferInstance.getLength(); i++) {
        bufferInstance.setLED(i, Color.kBlack);
      }
    }
}