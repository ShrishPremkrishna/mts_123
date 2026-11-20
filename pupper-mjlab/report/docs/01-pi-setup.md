# 1 · One-time Pi setup

Assumes the CS123 image: ROS 2 Jazzy, `~/pupperv3-monorepo` built, `robot.service` disabled.

## Repo

```bash
git clone https://github.com/ShrishPremkrishna/mts_123.git ~/mts_123
cd ~/mts_123 && git switch markus/opt-lab-2     # branch with these docs
```

The controller source used is `~/mts_123/pupper_gait_deploy`, our fork of `cs123-stanford/pupper_gait_deploy`.

## W&B login (once)

```bash
python3 -c "import wandb; print(wandb.__version__)"   # needs >= 0.22.3 for new 86-char keys
python3 -m pip install --user --upgrade "wandb>=0.22.3"   # only if older
python3 -m wandb login --relogin                     # paste key; account must read photon_mz-stanford-university
```

## Gamepad (DualSense)

Already paired: press **PS**. The light bar turns solid, and `ls /dev/input/js0` exists.

New pairing:
1. Hold **PS** for about 10 s until the light goes off.
2. Hold **Create + PS** for about 3 s until the light bar double-flashes. It is now in pairing mode.
3. On the Pi:
   ```bash
   bluetoothctl --timeout 15 scan on        # note the "Wireless Controller" MAC
   bluetoothctl pair <MAC> && bluetoothctl trust <MAC> && bluetoothctl connect <MAC>
   ```
`trust` is what makes it reconnect by itself next time.
