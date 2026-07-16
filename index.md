# Jamin's robotic arm project
My project is the robotic arm for arduino, a four jointed robotic arm that can pick items up with a claw gripper. The four joints are four servos that provide torque so that the specific joint can move accordingly to the manual controls. One of my biggest takeaways from this project I would say is the experience of the technical aspects from this project: assembling the hardware, and using the software.



| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Jamin J | College Champittet | Mechanical/software engineering | Entering final year of high school

![Cover image](Sonic_jam.png)

  
# Final Milestone


<iframe width="560" height="315" src="https://www.youtube.com/embed/6j2bav_Q7jk?si=4seBcMq0uvoBzZ-u" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


My final milestone was to add a arm movement speed regulator along with a position rest feature. The speed regulator is two buttons where one speeds up the movement of the arm and one slows it down wioth there being a total of 5 different speeds. The position reset features is a singular button that when pressed, resets the arm to its original position where all servos are at 90 degrees. My biggest challenge up to now is the handling of the wiring as prior to this project I have not had experience with this hardware. My biggest triumph up to now is getting the buttons to work with the breadboard as before I had tried to use the buttons integrated into the joysticks however I later found out that they were faulty so switched to buttons on a breadboard.



# Second Milestone

<iframe width="560" height="315" src="https://www.youtube.com/embed/ygS7DuomMOs?si=GDLErO7HQjZo27Qe" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

My second milestone was to add the "playback" feature to my robotic arm where I can record a specific movement of the arm and then replay it later. It works with two buttons, one to control the recording, and one to play the movement recording. The modification was mostly software based as it required coding for the playback feature, but the installation of the buttons also required wiring which did initially confuse me a little since it is the first time that I am using this hardware of the buttons, breadboard, and jumperwires. Something that would need to be completed is improvements upon the fluidity of the movements when being replayed as right now the robotic arm is able to mostly replay/repeat the moevements recorded at the same speed it was moved manually in an acceptable manner but it still has a little bit of a glitch in its movement which I will for sure fix.


# First Milestone

<iframe width="560" height="315" src="https://www.youtube.com/embed/Yc-WTwfIG3U?si=wUh1VxKrHHhFvJl6" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

My project is the robotic arm for arduino, to assemble a working, contollable robotic arm with a claw gripper. The parts that this project is composed of is the acrylic peices, the four servos, arduino nano shield, any many other smaller peices that make up and is required for the assembly of this project. The technical progress I have made so far in this project is assembling the physical arm, completing the wiring, and got the manual controls working. Challenges I am facing is one of the servos not providing enough torque causing it to be stuck when lowered too much.

# Schematics 

![Wiring Schematic](schematic.png)

# Code

```c++
/*
 * This code applies to cokoino mechanical arm
 * https://github.com/Cokoino/CKK0006
 *
 * Button behavior:
 *   Left press (1st): start recording
 *   Left press (2nd): stop recording
 *   Right press:      replay from recorded start position
 *   Left press (3rd): wipe + start recording again
 *   Left press (4th): stop recording
 *   ...repeats
 */
#include "src/CokoinoArm.h"
#define buzzerPin  3
#define btnLeft    8
#define btnRight   9
#define btnFaster  10
#define btnSlower  11
#define btnHome    12

CokoinoArm arm;
int xL, yL, xR, yR;

// Increase for longer recordings (each frame = 8 bytes; 50 = 400 bytes)
const int act_max = 50;
int act[act_max][4];
int startPos[4];
int num_do = 0;

// 0 = idle, 1 = recording, 2 = has recording
int recState = 0;

// Speed control: 5 levels, index 2 is the default (unchanged)
const int speedLevels[5] = {0, 1, 3, 7, 15};
int speedIndex = 2; // default = 20

unsigned long lastCapture = 0;
const unsigned long captureInterval = 500; // ms between captured frames

///////////////////////////////////////////////////////////////
void beep(int period_us, int duration_ms) {
  unsigned long end = millis() + duration_ms;
  while (millis() < end) {
    digitalWrite(buzzerPin, HIGH); delayMicroseconds(period_us);
    digitalWrite(buzzerPin, LOW);  delayMicroseconds(period_us);
  }
}
///////////////////////////////////////////////////////////////
void turnUD(void) {
  int s = speedLevels[speedIndex];
  if (xL != 512) {
    if (0   <= xL && xL <= 100) { arm.up(s);   return; }
    if (900  < xL && xL <=1024) { arm.down(s);  return; }
    if (100  < xL && xL <= 200) { arm.up(s);   return; }
    if (800  < xL && xL <= 900) { arm.down(s);  return; }
    if (200  < xL && xL <= 300) { arm.up(s);   return; }
    if (700  < xL && xL <= 800) { arm.down(s);  return; }
    if (300  < xL && xL <= 400) { arm.up(s);   return; }
    if (600  < xL && xL <= 700) { arm.down(s);  return; }
    if (400  < xL && xL <= 480) { arm.up(s);   return; }
    if (540  < xL && xL <= 600) { arm.down(s);  return; }
  }
}
///////////////////////////////////////////////////////////////
void turnLR(void) {
  int s = speedLevels[speedIndex];
  if (yL != 512) {
    if (0   <= yL && yL <= 100) { arm.right(s);  return; }
    if (900  < yL && yL <=1024) { arm.left(s);   return; }
    if (100  < yL && yL <= 200) { arm.right(s);  return; }
    if (800  < yL && yL <= 900) { arm.left(s);   return; }
    if (200  < yL && yL <= 300) { arm.right(s);  return; }
    if (700  < yL && yL <= 800) { arm.left(s);   return; }
    if (300  < yL && yL <= 400) { arm.right(s);  return; }
    if (600  < yL && yL <= 700) { arm.left(s);   return; }
    if (400  < yL && yL <= 480) { arm.right(s);  return; }
    if (540  < yL && yL <= 600) { arm.left(s);   return; }
  }
}
///////////////////////////////////////////////////////////////
void turnCO(void) {
  int s = speedLevels[speedIndex];
  if (xR != 512) {
    if (0   <= xR && xR <= 100) { arm.close(s);  return; }
    if (900  < xR && xR <=1024) { arm.open(s);   return; }
    if (100  < xR && xR <= 200) { arm.close(s);  return; }
    if (800  < xR && xR <= 900) { arm.open(s);   return; }
    if (200  < xR && xR <= 300) { arm.close(s);  return; }
    if (700  < xR && xR <= 800) { arm.open(s);   return; }
    if (300  < xR && xR <= 400) { arm.close(s);  return; }
    if (600  < xR && xR <= 700) { arm.open(s);   return; }
    if (400  < xR && xR <= 480) { arm.close(s);  return; }
    if (540  < xR && xR <= 600) { arm.open(s);   return; }
  }
}
///////////////////////////////////////////////////////////////
void date_processing(int *x, int *y) {
  if (abs(512 - *x) > abs(512 - *y)) { *y = 512; }
  else                                { *x = 512; }
}
///////////////////////////////////////////////////////////////
void checkButtons() {
  // --- LEFT BUTTON ---
  if (digitalRead(btnLeft) == LOW) {
    if (recState == 0 || recState == 2) {
      // Start (or restart) recording
      recState = 1;
      num_do = 0;
      int *p = arm.captureAction();
      for (int i = 0; i < 4; i++) startPos[i] = *(p + i);
      lastCapture = millis();
      beep(200, 150); // short high beep = recording started
    } else {
      // Stop recording
      recState = 2;
      beep(600, 300); // low beep = recording saved
    }
    while (digitalRead(btnLeft) == LOW) {} // wait for release
  }

  // --- CAPTURE FRAME while recording ---
  if (recState == 1 && millis() - lastCapture >= captureInterval) {
    if (num_do < act_max) {
      int *p = arm.captureAction();
      for (int i = 0; i < 4; i++) act[num_do][i] = *(p + i);
      num_do++;
    } else {
      // Buffer full — stop automatically
      recState = 2;
      beep(600, 300);
    }
    lastCapture = millis();
  }

  // --- RIGHT BUTTON: replay ---
  if (digitalRead(btnRight) == LOW && recState == 2) {
    beep(400, 100);
    // Move back to start position
    arm.servo1.write(startPos[0]);
    arm.servo2.write(startPos[1]);
    arm.servo3.write(startPos[2]);
    arm.servo4.write(startPos[3]);
    delay(500);
    // Replay each frame at the same interval it was recorded
    for (int i = 0; i < num_do; i++) {
      arm.servo1.write(act[i][0]);
      arm.servo2.write(act[i][1]);
      arm.servo3.write(act[i][2]);
      arm.servo4.write(act[i][3]);
      delay(captureInterval);
    }
    beep(200, 200);
    while (digitalRead(btnRight) == LOW) {}
  }

  // --- FASTER button ---
  if (digitalRead(btnFaster) == LOW) {
    if (speedIndex < 4) speedIndex++;
    beep(150, 80);
    while (digitalRead(btnFaster) == LOW) {}
  }

  // --- SLOWER button ---
  if (digitalRead(btnSlower) == LOW) {
    if (speedIndex > 0) speedIndex--;
    beep(600, 80);
    while (digitalRead(btnSlower) == LOW) {}
  }

  // --- HOME button: all servos to 90 ---
  if (digitalRead(btnHome) == LOW) {
    arm.servo1.write(90);
    arm.servo2.write(90);
    arm.servo3.write(90);
    arm.servo4.write(90);
    while (digitalRead(btnHome) == LOW) {}
  }
}
///////////////////////////////////////////////////////////////
void setup() {
  arm.ServoAttach(4, 5, 6, 7);
  arm.JoyStickAttach(A0, A1, A2, A3);
  pinMode(buzzerPin, OUTPUT);
  pinMode(btnLeft,    INPUT_PULLUP);
  pinMode(btnRight,   INPUT_PULLUP);
  pinMode(btnFaster,  INPUT_PULLUP);
  pinMode(btnSlower,  INPUT_PULLUP);
  pinMode(btnHome,    INPUT_PULLUP);
}
///////////////////////////////////////////////////////////////
void loop() {
  xL = arm.JoyStickL.read_x();
  yL = arm.JoyStickL.read_y();
  xR = arm.JoyStickR.read_x();
  yR = arm.JoyStickR.read_y();
  date_processing(&xL, &yL);
  date_processing(&xR, &yR);
  turnUD();
  turnLR();
  turnCO();
  checkButtons();
}
```

# Bill of Materials
| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| Cokoino CKK0006 4-Axis Robotic Arm Kit | Includes Nano, servos, joysticks, acrylic parts, screws | 58 USD | [Link](https://www.u-buy.ch/en/product/3PUQNSM-lk-cokoino-4-axis-robotic-arm-kit-for-arduino-4dof-mini-desktop-robot-arm-for-children-adults-compliment-engineering-math-science-and-technology-learn?srsltid=AfmBOoq4jKaJOkFeXI5C3a6njOb_Wvns5eEkLJdFX89tzTRQ_EQAxhTh&ref=hm-google-redirect) |
| PureCrea Prototype Shield V3 for Arduino Nano | Breaks out Nano pins for easier wiring | 15.9 USD | [Link](https://www.galaxus.ch/de/s1/product/purecrea-prototype-shield-v3-fuer-arduino-nano-entwicklungsboard-kit-36688810) |
| Duracell 9V Battery | Powers the servos | ~3 USD | [Link](https://www.coop-city.ch/de/wohnen-reisen/multimedia/batterien/alkaline-batterien/duracell-batterie-plus-9v6lr61-1-stueck/p/6761135) |
# Other Resources/Examples

- Robotic arm for arduino
- Github: https://github.com/Cokoino/CKK0006/tree/master



