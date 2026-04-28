# Robo Car Companion

A voice-controlled robot car built for Intro to Robotics, Spring 2026. You talk to it, it listens, and it does what you say — including driving toward you when you tell it to "come here."

## How It Works (Big Picture)

There are two ESP32 boards. One sits on the mic array and handles all the listening and voice recognition. When it hears a command, it sends it over Bluetooth to the second ESP32 on the car, which drives the motors accordingly.

---

## Files

### `robo_car_companion.ino`
This runs on the robot car itself. It connects to the mic array board over Bluetooth and waits for commands to come in. When it gets one, it moves:

- **Go Forward / Go Backward** — drives straight, using the onboard gyroscope to self-correct and not drift
- **Turn Left / Turn Right / Turn Around** — rotates to a precise angle using the gyroscope
- **Spin** — just spins in place until told to stop
- **Dance** — does a little routine (forward, back, spin both ways)
- **Come Here** — rotates to face whoever spoke, then drives toward them
- **Stop** — stops whatever it's doing

### `mic_array_BLE_and_inferencing.ino`
This runs on a second ESP32-S3 that's connected to an XMOS XVF3800 microphone array. It does the heavy lifting:

- Continuously listens to audio from the mic array
- Runs a machine learning model (trained with Edge Impulse) to recognize voice commands in real time
- Reads the direction the sound came from (so "Come Here" knows which way to turn)
- Sends recognized commands over Bluetooth to the car

It's set up to only fire a command once it's confident — it requires the model to agree with itself across multiple audio windows before sending anything.

### `mic_array_write_sound_to_serial.ino`
A helper sketch used during development. Instead of doing any inference, it just streams raw audio from the mic array to your computer over USB. This was used to collect training data for the voice model.

### `I2SAudioCollect.py`
A Python script that runs on your computer alongside `mic_array_write_sound_to_serial.ino`. It reads the audio coming in over serial and saves it as `.wav` files, organized into folders by label (e.g., "Dance", "Stop"). Used to build the dataset that the voice model was trained on.

### `micholder.STL`
A 3D-printable mount for the microphone array. Print it and attach it to the car.
