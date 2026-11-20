# Restore Pupper’s touchscreen from a Mac

Use this procedure when a previously enrolled Pupper FC on Raspberry Pi OS Bookworm boots to a terminal and the robot is unreachable over Wi-Fi. It uses the robot’s existing USB system disk. Saved wireless profiles stay on its Linux partition. The original eyes and touch Wi-Fi menu come from the installed Pupper GUI.

1. With the robot powered down, move its USB system disk to the Mac. Run `diskutil list external physical`. Identify the **whole device** by size and both partitions. The small FAT boot partition normally mounts at `/Volumes/bootfs`. The second partition contains the Pi’s Linux system. Decline any macOS offer to initialize the Linux partition.
2. From this checkout, substitute the **current** boot partition identifier for `diskNs1` below. macOS can assign a different number each time the disk is connected. The tool verifies that the mount path and identifier refer to the same external FAT partition. `status` verifies the staged files and should print `Hook armed`.

   ```sh
   python3 foundation/hardware/fc_usb_touch_rescue.py stage /Volumes/bootfs --device diskNs1
   python3 foundation/hardware/fc_usb_touch_rescue.py status /Volumes/bootfs --device diskNs1
   ```

3. Eject the matching **whole disk** with `diskutil eject /dev/diskN`. Wait for eject confirmation, put the disk back in the powered-down Pi, then power on. Allow a second boot. The one-shot repair restores the graphical boot target while keeping the appliance marker and robot, course-robot and LLM agent startup guards in place. It starts the display and network services and requests a normal reboot once it has checked them.
4. On the Pi, look for the original eyes, touch the screen and open **WiFi → Enter pairing mode**. Follow the screen instructions with a phone. Confirm the Pi actually joins the chosen network and that the laptop can reconnect. Seeing the display, registering a touch and joining Wi-Fi are physical checks.

## Inspect or cancel

Before booting the Pi, `cancel` restores the exact original `cmdline.txt` and removes an unused rescue folder. Use the same partition selection as in step 2.

```sh
python3 foundation/hardware/fc_usb_touch_rescue.py cancel /Volumes/bootfs --device diskNs1
```

If the screen still shows a terminal, power down before moving the USB disk back to the Mac. Use `status` to read `fc-touch-rescue/result.json` on the FAT partition. The receipt reports the selected service states, display socket, GUI process stability and whether the original kernel command line was restored. SSH and NetworkManager are requested on failure. A network connection also requires an available network and usable credentials.

If power is lost before the graphical configuration and motion guards are verified and synced, the one-shot remains armed for the next boot. Once those persistent settings are verified, the original kernel command line is restored. A subsequent display-service timeout leaves the diagnostic receipt and the graphical settings in place. Canceling after a repair retains its receipt. Leave the FC appliance boot marker in place to keep legacy robot autostart suppressed.

The hook uses Bookworm’s `systemd-run-generator` and the Pi firmware’s kernel command line on the FAT partition. [systemd v252 generator](https://github.com/systemd/systemd/blob/v252/src/run-generator/run-generator.c) · [Raspberry Pi boot configuration](https://www.raspberrypi.com/documentation/computers/config_txt.html)
