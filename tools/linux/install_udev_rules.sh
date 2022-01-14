#!/bin/sh -x
sudo cp bst_coines.rules /etc/udev/rules.d/bst_coines.rules
sudo cp 99-usb_bootloader.rules /etc/udev/rules.d/99-usb_bootloader.rules
sudo udevadm control --reload-rules

