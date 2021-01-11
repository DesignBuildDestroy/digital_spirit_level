/* DesignBuildDestroy.com - Digital Level 1.0
   Written by DesignBuildDestroy (DBD) 2020

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Required Libraries
   * Jeff Rowberg's MPU6050 library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
   * Jeff Rowberg's I2Cdev library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
   * Adafruit SSD1306 OLED library: https://github.com/adafruit/Adafruit_SSD1306
   * Adafruit GFX library: https://github.com/adafruit/Adafruit-GFX-Library
   * Adafruit font from GFX library: https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Fonts/FreeMono9pt7b.h
   
 */

#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"   //Must include for DMP holds firmware hex to push to MPU on init
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <EEPROM.h>
#include <Fonts/FreeMono9pt7b.h> //Font for menus only


//  SSD1306 OLED
const byte SCREEN_WIDTH = 128; // OLED display width, in pixels
const byte SCREEN_HEIGHT = 64; // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//  Menu & menu button definitions
const byte MENU_BTN = 3;
const byte ENTER_BTN = 4;
unsigned long menuStartTime = 0;      //To use with menuTimeout to track if we should auto exit
byte menuItem = 0;     //Hold the current menu's item selected
const int menuTimeout = 10000; //Time out value for menus to auto exit
bool precisionMode = false; //Menu option to show 1 decimal places instead of rounding to whole.


// MPU Address for I2C
byte devAddr = 0x68;
MPU6050 mpu(devAddr);

// MPU control/status vars from JRowberg
bool dmpReady = false;  // set true if DMP init was successful
byte mpuIntStatus;   	// holds actual interrupt status byte from MPU
byte devStatus;      	// return status after each device operation (0 = success, !0 = error)
int packetSize;    		// expected DMP packet size (default is 42 bytes)
int fifoCount;     		// count of all bytes currently in FIFO
byte fifoBuffer[64]; 	// FIFO storage buffer
Quaternion q;           // [w, x, y, z] quaternion container


// Modified version of Adafruit BN0555 library to convert Quaternion to world angles the way we need
// The math is a little different here compared to Adafruit's version to work the way I needed for this project
VectorFloat QtoEulerAngle(Quaternion qt) {
  VectorFloat ret;
  double sqw = qt.w * qt.w;
  double sqx = qt.x * qt.x;
  double sqy = qt.y * qt.y;
  double sqz = qt.z * qt.z;

  ret.x = atan2(2.0 * (qt.x * qt.y + qt.z * qt.w), (sqx - sqy - sqz + sqw));
  ret.y = asin(2.0 * (qt.x * qt.z - qt.y * qt.w) / (sqx + sqy + sqz + sqw));  //Adafruit uses -2.0 *..
  ret.z = atan2( 2.0 * (qt.y * qt.z + qt.x * qt.w), (-sqx - sqy + sqz + sqw));

  // Added to convert Radian to Degrees
  ret.x = ret.x * 180 / PI;
  ret.y = ret.y * 180 / PI;
  ret.z = ret.z * 180 / PI;
  return ret;
}

// Write a 2 byte word starting at the startAddr provided
void epromWriteWord(int startAddr, int value) {
  EEPROM.update(startAddr, value);
  EEPROM.update(startAddr+1, value >> 8);
}

// Return a 2 byte word read from a starting EEPROM address
int epromReadWord(int startAddr) {
  int value = EEPROM.read(startAddr);
  value = value | (EEPROM.read(startAddr+1) << 8);
  return value;
}

void getCalibration() {
  // Get the saved calibration values from EEPROM and update MPU
  // Address Always starts at 0 ends at 11, 2 bytes per axis

  // Future Check if we have anything saved (look for all FF)
  // if not assume calibration not run and skip setting just MPU default
  // Eventually prompt user to calibrate! if there's enough space left in memory of this sketch!!!
    mpu.setXAccelOffset(epromReadWord(0));
    mpu.setYAccelOffset(epromReadWord(2));
    mpu.setZAccelOffset(epromReadWord(4));
    mpu.setXGyroOffset(epromReadWord(6));
    mpu.setYGyroOffset(epromReadWord(8));
    mpu.setZGyroOffset(epromReadWord(10)); // last address read would be 11 decimal in eeprom    
}

void setCalibration() {
  // Run DMP auto calibration and then get those values and save to EEprom
  // This should only be called when Auto Calibrate option is selected
  // to preserve EEPROM life use update instead of write

  // Run autocalibration 6 times
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  // Get the final values that were saved and move them to EEPROM
  int Data[3];
  // Accel Offsets
  I2Cdev::readWords(devAddr, 0x06, 3, (int *)Data);
  epromWriteWord(0,Data[0]);
  epromWriteWord(2,Data[1]);
  epromWriteWord(4,Data[2]);
  // Gyro Offsets
  I2Cdev::readWords(devAddr, 0x13, 3, (int *)Data);
  epromWriteWord(6,Data[0]);
  epromWriteWord(8,Data[1]);
  epromWriteWord(10,Data[2]); // Last byte written is eeprom address 11 decimal 
}



void setup() {
  // From JRowber sample to setup MPU6050 connection
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200); // Only needed for Debug otherwise can comment out

  // Set menu buttons WITH PULLUPS
  pinMode(MENU_BTN, INPUT_PULLUP);
  pinMode(ENTER_BTN, INPUT_PULLUP);

  // Init OLED Display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE); //not really necessary for this display
  display.println();
  display.println(F("Starting!!"));
  display.display();


  // Init MPU6050
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // Get stored EEPROM Calibration values and send to MPU
  // Otherwise default to predefined and display Calibration needed!
  getCalibration();

  // make sure it worked - Because we are pushing firmware on startup of DMP
  // we need to check that everything actually went as planned devStatus 0 is success.
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

  } else {
    // IMU Failed for some reason show error on OLED
    display.clearDisplay();
    display.setCursor(0, 40);
    display.println("IMU FAIL");
    display.display();
  }
}


// Handles display routine for Main menu
void dispMenu(byte itemSelect) {
  display.clearDisplay();
  display.setRotation(0);
  display.setFont(&FreeMono9pt7b);
  display.setCursor(0, 14);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.println(F("   Normal"));
  display.println(F(" Precision"));
  display.println(F(" Calibrate"));

  if (itemSelect == 0) {
    display.drawRect(5, 2, 120, 17, SSD1306_WHITE); // Draw box around item 1
  }
  if (itemSelect == 1) {
    display.drawRect(5, 18, 120, 20, SSD1306_WHITE); // Draw box around item 2
  }
  if (itemSelect == 2) {
    display.drawRect(5, 36, 120, 20, SSD1306_WHITE); // Draw box around item 3
  }

  // Display everything
  display.display();
}

// Handles display routine for Calibration sub menu
void dispCalibrate(byte itemSelect) {
  display.clearDisplay();
  display.setFont(&FreeMono9pt7b);
  display.setCursor(2, 14);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.println(F("Lay me down"));
  display.println(F("  face up"));
  display.setCursor(0, 55);
  display.println(F(" START   X"));

  if (itemSelect == 0) {
    display.drawRect(6, 41, 65, 20, SSD1306_WHITE); //Draw box around START
  }
  else  {
    display.drawRect(95, 41, 20, 20, SSD1306_WHITE); //Draw box around X
  }

  // Display everything
  display.display();
}

// Handle user button interactions for the main menu options
void menuMainWait() {
  // Show the main Menu text on screen
  menuItem = 0;
  dispMenu(menuItem);

  // Wait for key press to go to selected item
  menuStartTime = millis(); //Reset menu timout

  // Wait for next user action in the menu, if no buttons pressed
  // after menuTimeout limit, auto exit the menu
  while (millis() - menuStartTime <= menuTimeout) {
    if (digitalRead(MENU_BTN) == LOW) {
      delay(200); // Debounce
      menuStartTime = millis(); // Reset menu timout
      menuItem++;
      if (menuItem >= 3) {
        // We reached menu item limit, roll around to start
        menuItem = 0;
        dispMenu(menuItem);
      }
      else {
        // Draw rect around next menu item selected
        dispMenu(menuItem);
      }
    }
    if (digitalRead(ENTER_BTN) == LOW) {
      delay(200); // Debounce
      menuStartTime = millis(); // Reset menu timout
      if (menuItem == 0) {
        // Normal mod,e Angle is rounded to whole number
        precisionMode = false;
        break;
      }
      if (menuItem == 1) {
        // Precision mode, Angle rounded to 1st decimal place
        precisionMode = true;
        break;
      }
      if (menuItem == 2) {
        // Show calibration sub menu
        menuCalibrateWait();
        break;
      }
    }
  }
}

// Auto Calibration menu, will use MPU auto calibration feature
// and save the values to EEPROM that are pulled on startup
// Calibration only needs to be run one time and values should stay
// acurate from then on.
// NOTE: Calibration must be done on an already known LEVEL surface that
//  is level front to back and side to side like a level table top with the device laying down
//  screen facing up.

void menuCalibrateWait() {
  // Show the sub menu for CALIBRATION
  menuItem = 1; // Default to EXIT
  dispCalibrate(menuItem);

  // Wait for next user action in the menu, if no buttons pressed
  // after menuTimeout limit, auto exit the menu
  menuStartTime = millis(); // Reset menu start timer
  while (millis() - menuStartTime <= menuTimeout) {
    if (digitalRead(MENU_BTN) == LOW) {
      delay(300); // Debounce
      if (menuItem == 1) {
        menuItem = 0;
        dispCalibrate(menuItem);
      }
      else {
        menuItem = 1;
        dispCalibrate(menuItem);
      }
    }
    if (digitalRead(ENTER_BTN) == LOW) {
      delay(200); //Debounce
      if (menuItem == 0) {
        // Start Calbiration process takes a few seconds to run
        // So display something to let user know it's working
        display.clearDisplay();
        display.setCursor(0, 40);
        display.println("CALIBRATING");
        display.display();

        setCalibration(); // This runs the actual calibration
        
        display.clearDisplay();
        display.setCursor(0, 40);
        display.println("COMPLETE!");
        display.display();
        delay(500);  // Give user time to see complete message before exit
        break;
      }
      if (menuItem == 1) {
        // X selected - Exit Menus;
        break;
      }
    }
  }
}


// For OLED, show the angle at the correct orientation based on
// How the level is positioned, whether Precision mode is set
// changes decimal display and font size to fit characters
void formatDisplay(double angleVal, byte dispRotate) {

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setFont();
  display.setRotation(dispRotate);

  // Set font size, text position and angle value based on MPU positions and precision setting
  // Precision mode shows 1 decimal place value, otherwise round to whole number
  if (precisionMode == false) {
    if (dispRotate == 0 || dispRotate == 2) { 
      // Horizontal
      display.setCursor(40, 10);
    }
    else { 
      // Vertical
      display.setCursor(3, 30);
    }

    display.setTextSize(5);
    display.println(round(abs(angleVal)));
  }
  else { 
    // Precision Mode show 1 decimal place smaller font when vertical
    if (dispRotate == 0 || dispRotate == 2) { 
      // Horizontal
      display.setTextSize(4);
      display.setCursor(20, 10);
    }
    else { 
      // Vertical
      display.setTextSize(2);
      display.setCursor(6, 40);
    }

    display.println(abs(angleVal), 1);
  }

  // If we are level or plumb +-1 degree Display it under the angle
  // again format for rotation position use LEVEL or PLUMB text indicator
  // Do this inside a white rectangle with black font so it stands out
  // since it will be very small font size
  if (round(abs(angleVal)) <= 1) {
    display.setTextSize(1);
    display.setTextColor(BLACK, WHITE);
    if (dispRotate == 0 || dispRotate == 2) { 
      // Horizontal
      display.setCursor(37, 50);
      display.println(F("   LEVEL  "));
    }
    else { 
      // Vertical
      display.setCursor(0, 90);
      display.println(F("   PLUMB  "));
    }
  }

  // Reset text color
  display.setTextColor(WHITE, BLACK); 
  // Show the text
  display.display();
}


// MAIN PROGRAM LOOP!!
void loop() {
  if (digitalRead(MENU_BTN) == LOW) {
    delay(500);
    menuMainWait();
  }
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // Get the Quaternion values from DMP buffer
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    // Calc angles converting Quaternion to Euler this was giving more stable acurate results compared to
    // getting Euler directly from DMP. I think Quaternion conversion takes care of gimble lock.
    VectorFloat ea = QtoEulerAngle(q);

    //DEBUG ONLY COMMENT OUT UNLESS NEEDED
    /*  Serial.print("quat\t");
      Serial.print(ea.x);
      Serial.print("\t");
      Serial.print(ea.y);
      Serial.print("\t");
      Serial.print(ea.z);
      Serial.println("\t"); 
    */
    
    float angVal = 0;
    float dispRotate = 0;

    // Figure out how to display the data on OLED at the right rotation
    // Like flipping a phone around rotating the display - trial and error...took a while
    // TOP IS TOP means TOP of OLED is the TOP of the screen, RIGHT IS TOP Right side of OLED is now the top, etc.

    // TOP IS TOP angling Right side (LEVEL)
    if (ea.x > 0 && ea.y > 35 && ea.y <= 90) {
      angVal = (90 - ea.y);
      dispRotate = 0;
    }
    // Angling right side up RIGHT is TOP (toward PLUMB)
    if (ea.x > 0 && ea.y <= 35 && ea.y > -35) {
      angVal = ea.y;
      dispRotate = 1;
    }
    // LEFT IS TOP (PLUMB)
    if (ea.x > 0 && ea.y <= -35 && ea.y > -90) {
      angVal = ea.y + 90;
      dispRotate = 2;
    }
    // TOP IS TOP angling Left side (LEVEL)
    if (ea.x < 0 && ea.y > 35 && ea.y <= 90) {
      angVal = (90 - ea.y);
      dispRotate = 0;
    }
    // Upside down - BOTTOM IS TOP (LEVEL)
    if (ea.x < 0 && ea.y <= 35 && ea.y > -35) {
      angVal = ea.y;
      dispRotate = 3;
    }
    // Upside down - BOTTOM IS TOP
    if (ea.x < 0 && ea.y <= -35 && ea.y > -90) {
      angVal = ea.y + 90;
      dispRotate = 2;
    }
    // laying down face up - this is also Calibration position
    // need to also check Z here or it can get confused with another position
    if (ea.x < 5 && ea.x > -20 && ea.y <= 35 && ea.y > -35 && ea.z < 5) {
      angVal = 90 - ea.y;
      if (angVal > 50) {
        angVal -= 90; // bandaid fix from being laid flat...
      }
      dispRotate = 0;
    }

    // Display the data on OLED formatted as we need for the position
    formatDisplay(angVal, dispRotate);
  }
}
