# Linux Surface Camera Support

A dump of notes and code to try to add support for the Surface Book 2 cameras to
Linux. Until I change this readme, this is all very experimental and haphazard.

This is provided with absolutely no guarantees of anything.

# Get the code

## Option 1: Use my private kernel tree

The easiest way to pull the code is to use my kernel fork at
https://github.com/jhand2/linux and checkout the `sb2_ov5693` branch. You can
also use the config in this repo, `config-5.6-surface`.

My branch is based on kernel 5.6 and also contains all the patches for 5.6
from the [linux-surface/linux-surface](
https://github.com/linux-surface/linux-surface) repo.

## Option 2: Apply patches from this repo

You can also simply apply the patches in the `patches` directory in this repo.
They are kept in sync with the above fork. If you choose to do this, you should
also apply the patches from the linux-surface repo.

# Testing

I have included a script camtest.sh for testing the drivers. It is primarily
based on the [IPU3 documentation from kernel.org](
https://www.kernel.org/doc/html/latest/media/v4l-drivers/ipu3.html).

You may need to pull and compile the yavta tool from
git://git.ideasonboard.org/yavta.git.

Currently this script hangs on the final command because no image buffers
are supplied. For better troubleshooting, I recommend running the last command
under strace.
