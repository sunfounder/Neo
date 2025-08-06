# Python Library for the Neo

---
## Installation

### dependencies

- robot_hat (<https://github.com/sunfounder/robot-hat>)

  Hat driver library for Raspberry Pi, includes the PWM, ADC, Servo, and Motor control，and etc.

```bash
cd ~
git clone -b v2.0 https://github.com/sunfounder/robot-hat.git --depth=1
cd robot-hat
sudo python3 setup.py install
```

- i2samp.sh

  A script to install the robothat sound card driver.

```bash
cd ~/robot-hat
sudo bash i2samp.sh
```

- vilib (<https://github.com/sunfounder/vilib>)

  image visual processing library for Raspberry Pi.

```bash
cd ~
git clone https://github.com/sunfounder/vilib.git --depth=1
cd vilib
sudo python3 install.py
```

- Neo (this)

```
cd ~
git clone https://github.com/sunfounder/Neo.git --depth=1
cd Neo
sudo python3 setup.py install

```
---
## Basic examples
  Simple usage examples of all Neo modules and sensors.
  </br><a href="./example" >basic examples</a>

## Hailo
  Usage of the Hailo module
  <a href="./hailo" >hailo usage</a>

## Gpt example
  A project that uses ChatGPT API to implement real-time conversations and provide action feedback
  </br><a href="./gpt_example" >gpt_example</a>

## Auto-drive example
  A project that simulates auto driving by detecting lane lines through camera vision
  </br><a href="./auto_driver_example" >auto_driver_example</a>


---
## Data sheet
https://github.com/sunfounder/Neo/tree/data_sheet