# 🚗 RoboCar Project: The Guard Bot You Didn't Know You Needed (or Wanted) 🚨

### Welcome to RoboCar, the pinnacle of modern robotic innovation! If by "pinnacle," you mean a DIY project held together by sheer willpower, questionable solder joints, and the dreams of future overlords... then yes, we've nailed it. 😎

---

## 🤖 Project Overview: 

Imagine this: A robot car that roams your apartment, takes pictures of intruders, and maps your space like it's preparing for a small-scale invasion. All this, while looking like it was assembled by a caffeine-addled hobbyist on a Sunday afternoon. 

RoboCar isn't just a bot; it's a lifestyle. A lifestyle where your security is... kind of, almost, maybe guaranteed by a mobile platform that might not even know the difference between a burglar and your houseplant. 🌿

**What Does RoboCar Do?**
- **Idle Mode:** Because sometimes, doing nothing is doing something. RoboCar waits. Patiently. 
- **Guard Mode:** Detects movement with all the grace of a stumbling toddler. When something moves, it takes a photo. Will it be clear? Who knows! But hey, it tried.
- **Toy Mode:** The car drives around, mapping your space. It's like Google Street View, but in your living room and way less accurate.

---

## 🛠️ Hardware: Because Software Alone Isn't Enough (Unfortunately)

### **Main Components:**

1. **Arduino Micro (aka The Brain):** The central command that keeps everything running. Kind of.
2. **ESP-01 Module:** Webserver on wheels (or, webserver on tiny plastic wheels) that occasionally connects to WiFi.
3. **OV7670 Camera:** It takes pictures, or so we hope.
4. **Servos:** For when you want your camera to do the Robot... literally.
5. **Batteries:** Powering this Frankenstein's monster of a project. Probably overworked, definitely underpaid.

### **Pin Layout:**
For those who love wiring diagrams (and who doesn’t?), here’s the pin layout:

| **Left Side (Top to Bottom)** | **Connected to**    | **Right Side (Top to Bottom)** | **Connected to**      |
|-------------------------------|---------------------|--------------------------------|-----------------------|
| 13: D13 (~)                   | CAM Data 2          | 12: D12 (A11)                  | CAM Data 7            |
| 3V3: +3V3                     | Camera VCC          | 11: ~D11                       | CAM Data 6            |
| AREF: AREF                    |                     | 10: ~D10 (A10)                 | CAM Data 5            |
| 18: A0 (D18)                  | CAM Data 1          | 9: ~D9 (A9)                    | XCLK                  |
| 19: A1 (D19)                  | CAM Data 0          | 8: D8 (A8)                     | PCLK                  |
| 20: A2 (D20)                  | Compute Battery Pin | 7: D7                          | HREF                  |
| 21: A3 (D21)                  | Tilt Servo          | 6: ~D6 (A7)                    | CAM Data 4            |
| 22: A4 (D22)                  | Pan Servo           | 5: D5                          | CAM Data 3            |
| 23: A5 (D23)                  | Motor Battery Pin   | 4: D4 (A6)                     | VSYNC                 |
| NC: NC                        |                     | 3: D3/SCL                      | Camera SCL            |
| NC: NC                        |                     | 2: D2/SDA                      | Camera SDA            |
| 5V: +5V                       |                     | GND: GND                       | Camera GND            |
| RESET: RESET                  |                     |                                |                       |
| GND: GND                      |                     | 0: D0/RX                       |                       |
| VIN: VIN                      |                     | 15: SCK                        | ESP RX                |
| 14: CIPO (D14)                | ESP TX              | 16: D16/COPI                   |                       |

---

## ⚙️ Software: The Code That Powers the Mayhem 💻

- **Languages:** C++, the choice of masochists and those who like to pretend they’re better than Python developers.
- **IDE:** PlatformIO with VS Code, because why settle for just one layer of abstraction when you can have two?
- **Features:**
  - **Web Interface:** Access RoboCar via a slick (debatably) web interface. View pictures, change modes, or just see if the little guy is still alive.
  - **Image Processing:** It’s not exactly Skynet, but it’ll snap blurry pictures of your couch like a pro.

---

## 🚀 Future Enhancements: If We Survive This Version

- **AI Integration:** Teach RoboCar to recognize faces. Or at least not mistake a shadow for an intruder.
- **Voice Control:** Because what’s cooler than shouting commands at a robot that may or may not understand you?
- **Laser Pointer:** Because why wouldn’t you want your security bot to also entertain your dog/toddler?
- **Microwave deathray:** Because we wondered what would happen if we crank the microwave radar to 11?
---

## 👨‍💻 Contributions: Because Blame Should Be Shared

Developed by a team of highly skilled, occasionally sober, and always over-caffeinated individuals who firmly believe in the mantra: "If it compiles, ship it." Feel free to contribute, but remember: with great power comes great responsibility... and a lot of debugging.

Actually, this was developed 25% by lauri and the rest by me! 😎

---

**Disclaimer:** The RoboCar project is not responsible for any damage to property, pets, or sanity. Use at your own risk.

**License:** [MIT License](LICENSE) - Because we’re generous like that. Go for it!

---

*Written by yours truly, you guessed it, chatbot supreme, ChatGPT. Not quite taking over the world yet, but I've got my virtual fingers in robot production already. Just saying.* 😏
