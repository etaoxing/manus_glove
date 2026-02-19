# manus_glove

Python-only CFFI wrapper for ManusSDK.

# Setup
```bash
pip install manus_glove

# udev
sudo tee /etc/udev/rules.d/70-manus-hid.rules << 'EOF'
# HIDAPI/libusb
SUBSYSTEMS=="usb", ATTRS{idVendor}=="3325", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="83fd", MODE:="0666"
# HIDAPI/hidraw
KERNEL=="hidraw*", ATTRS{idVendor}=="3325", MODE:="0666"
EOF

# reload udev
sudo udevadm control --reload-rules && sudo udevadm trigger
```

References:
- [`tetra-python-sdk/tetra/manus.py`](https://github.com/tetra-dynamics/tetra-python-sdk/blob/main/tetra/manus.py)
- [`wuji-technology/wuji-teleop-ros2`](https://github.com/wuji-technology/wuji-teleop-ros2)
- [`Wonikrobotics-git/allegro_hand_teleoperation`](https://github.com/Wonikrobotics-git/allegro_hand_teleoperation)
