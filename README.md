# Python Library for the ZeusPi

## Installation

### dependencies

- robot_hat (<https://github.com/sunfounder/robot-hat>)

  Hat driver library for Raspberry Pi, includes the PWM, ADC, Servo, and Motor control，and etc.

```bash
cd ~
git clone https://github.com/sunfounder/robot-hat.git --depth=1
cd robot-hat
sudo python setup.py install
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
sudo python install.py
```

- ZeusPi (this)

```
cd ~
git clone <https://github.com/sunfounder/ZeusPi.git> --depth=1
cd ZeusPi
sudo python setup.py install

```
