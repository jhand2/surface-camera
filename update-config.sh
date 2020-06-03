#!/bin/bash

KERNEL_SRC=$HOME/code/linux

pushd $KERNEL_SRC

./scripts/kconfig/streamline_config.pl > config_strip

popd
