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
# Basic examples


# Gpt example

# Auto-drive example
  <a href="./auto_driver_example" >auto_driver_example</a>


---
# Data sheet
https://github.com/sunfounder/Neo/tree/data_sheet