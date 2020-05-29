# Notes about adding support for the Surface Book 2 webcam

## Upstream Work
* IPU3: https://github.com/torvalds/linux/tree/master/drivers/staging/media/ipu3

## Reference docs
* IPU: https://www.kernel.org/doc/html/latest/media/v4l-drivers/ipu3.html
* Data sheets for OV drivers: https://github.com/Doridian/sb2-linux-cameras

# Different components Involved
Note, the SB2 has both front and back cameras and likely requires different
drivers for each.

* OV5693
    * A camera sensor with an I2C interface
    * I2C interface is used for control
    * Existing OV drivers are upstream at drivers/media/i2c
    * Similar to the ov5695?
    * Similar to the ov5675?
    * Front camera
* OV8865
    * Similar to the ov8856?
    * Rear camera
* OV7251
    * Already upstream
    * Front infrared camera
* Camera Interface
    * Interface is MIPI CSI-2
    * PCI device ID (8086:9d32)
    * All the cameras share this device. It delivers the images
    * CSI-2 interface is used for image delivery
* INT3472 devices
    * Potentially for flash and LEDs
* Image Signal Processor
    * PCI device ID (8086:1919)
    * IPU3 driver for this device is upstream at drivers/staging/media/ipu3
    * Driver is available in linux-surface kernel

## Instructions

### Building the i2c media drivers against the running kernel

```
make -j`nproc` -C /lib/modules/5.6.14-surface/build M=/home/jorhand/code/linux/drivers/media/i2c modules
```
