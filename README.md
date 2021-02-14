# Raspberry Pi car

For the [SunFounder PiCar-V Kit V2.0 for Raspberry Pi](https://www.sunfounder.com/collections/robotics/products/smart-video-car).
See also the [Sunfounder Smart Video Car Kit for RaspberryPi](https://github.com/sunfounder/Sunfounder_Smart_Video_Car_Kit_for_RaspberryPi)
GitHub page.

## Preparing SD card

1. Get the latest Raspbian (Buster) Lite image from the
    Raspberry Pi
    [Download page](https://www.raspberrypi.org/downloads/raspbian/).
2. Unzip the zipped image file in your downloads folder:

    ```bash
unzip 2021-01-11-raspios-buster-armhf-lite.zip
    ```

3. Check the block device file name assigned to the MicroSD card:

    ```bash
lsblk -p
    ```

4. Write the Raspbian image to the SD card:

    ```bash
sudo dd \
    bs=1M status=progress \
    if=2021-01-11-raspios-buster-armhf-lite.img \
    of=/dev/mmcblk0
    ```

5. Mount the newly written file systems on the SD using your preferred file
    system browser. Once the file systems are mounted, export the common path
    part to a variable. We'll continue to use this variable in the examples
    below. Of course these commands can be tailored to your particular setup.

    ```bash
# e.g. for /path/to/boot
export MOUNT=/path/to
    ```

6. Pre-configure the wifi module by placing the wpa_supplicant.conf file in the
    boot partition:

    ```bash
export SSID="your-wifi-name"
export PSK="your-wifi-password"
export COUNTRY="US"

cat <<EOF > $MOUNT/boot/wpa_supplicant.conf
country=$COUNTRY # Your 2-digit country code
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
$(wpa_passphrase $SSID "$PSK")
EOF

cat <<EOF | sudo tee $MOUNT/rootfs/etc/network/interfaces.d/wlan0.conf
auto wlan0
allow-hotplug wlan0
iface wlan0 inet manual
wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
EOF
    ```

7. Enable the ssh service on boot, by creating the `ssh` file in the boot
    partition:

    ```bash
touch $MOUNT/boot/ssh
    ```

8. Enable password-less login to the `pi` user:

    ```bash
mkdir \
    --mode=700 \
    $MOUNT/rootfs/home/pi/.ssh
cp \
    ~/.ssh/id_rsa.pub \
    $MOUNT/rootfs/home/pi/.ssh/authorized_keys
chmod 600 $MOUNT/rootfs/home/pi/.ssh/authorized_keys
    ```

9. Disable the default `pi` password:

    ```bash
sudo sed -ri \
    's/^pi:[^:]+:/pi::/' \
    $MOUNT/rootfs/etc/shadow
    ```

    Although this step is optional, it is highly recommended. Alternatively you
    can change the default password once you're logged in.

10. Change the default hostname:

    ```bash
sudo sed -ri \
    's/raspberrypi/raspicar/g' \
    $MOUNT/rootfs/etc/hostname \
    $MOUNT/rootfs/etc/hosts
    ```

## Setting up

### Installing dependencies

A fresh, lite RaspbianOS doesn't come with all the tools installed:

```bash
sudo apt update
sudo apt install \
    vim \
    git \
    python3-venv \
    python3-dev \
    python3-pyqt5 \
    build-essential \
    cmake \
    pkg-config \
    libjpeg-dev \
    libtiff5-dev \
    libjasper-dev \
    libpng-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libfontconfig1-dev \
    libcairo2-dev \
    libgdk-pixbuf2.0-dev \
    libpango1.0-dev \
    libgtk2.0-dev \
    libgtk-3-dev \
    libatlas-base-dev \
    gfortran \
    libhdf5-dev \
    libhdf5-serial-dev \
    libhdf5-103 \
    libqtgui4 \
    libqtwebkit4 \
    libqt4-test \
    libilmbase-dev \
    libopenexr-dev \
    libgstreamer1.0-dev
```

### Upgrade the firmware

Upgrade the firmware to get the latest module support and performance upgrades:

```
sudo rpi-update
```

### Enabling the picamera module and i2c

```bash
sudo raspi-config
```

Then go to:
* `Interface Options > Camera`
    * Enable the camera interface by choosing `<Yes>`
* `Interface Options > I2C`
  * Enable `ARM I2C` by choosing `<Yes>`
Finally reboot the Pi

## Quick start

### Setup a Python environment

In a fresh check-out of this repository:

```bash
python3 -m venv .raspicar
. .raspicar/bin/activate
pip install -r requirements.txt
```

### Scan a panorama

Bootstrap the configuration:

```bash
cat <<EOF > config.json
{
    "camera": {
    },
    "car": {
        "max_speed_rpm": 180.0,
        "max_rotation_angle": 40.0,
        "vehicle_length": 140.0,
        "wheel_diameter": 65.0
    },
    "panorama": {
        "steps": 6,
        "pan": [-0.2, 0.2],
        "tilt": [0.6, 0.8],
        "delay": 3.0,
        "directory": "panorama"
    }
}
EOF
```

Scan the room. Make sure a print-out of the
[calibration pattern](tools/acircles_pattern.png) is in view for all shots.

```bash
./panorama.py --config config.json
```
