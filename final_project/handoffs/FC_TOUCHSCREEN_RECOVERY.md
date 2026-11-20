# Touchscreen boot and USB recovery

The optional boot selector keeps the graphical Pupper eyes and touch Wi-Fi. It journals the previous boot target and gates competing services on the next boot. Its `normal` command restores the saved configuration. The separate USB tool stages, inspects or cancels a one-time repair on an explicitly selected external Pi boot partition.

Run `python -m pytest foundation/hardware/test_fc_appliance_conditions.py foundation/hardware/test_fc_usb_touch_rescue.py`. These exercise normal and interrupted recovery using temporary filesystems. The operator previously observed the eyes and touch Wi-Fi return on Blue after a USB repair.

When qualifying another Pi image, inventory its graphical and network units, preview the selector with `enable --dry-run --root DIR` against the mounted root, and verify both the eyes and tap-to-reconnect across a reboot. Record the original target and verify the return to normal mode before adopting the profile for that image. Merging these tools does not activate a boot profile.
