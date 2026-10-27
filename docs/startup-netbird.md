# Derek startup: power on to NetBird

## 1. Start Derek and connect Wi-Fi

Turn on Derek's main power and motor power. Let Ubuntu finish starting.
On Derek's screen, connect to **eduroam** using the saved university login.
The Mac can stay on university Wi-Fi or another internet connection.

On Derek, check:

```bash
nmcli device status
```

`wlan0` should show eduroam. `HW-24347889` is Derek's own hotspot;
being connected to that hotspot does not give Derek internet.

## 2. Check the clock

Derek's date has reset after restarting. An incorrect date can prevent
NetBird login and secure connections.

Run on Derek:

```bash
sudo timedatectl set-ntp true
sudo systemctl restart systemd-timesyncd
timedatectl
```

Check that the date is correct and these lines appear:

```text
System clock synchronized: yes
NTP service: active
```

Time synchronization needs internet. If the date stays wrong, correct it
in Ubuntu's Date & Time settings using the current date and time, then
check synchronization again. Hong Kong time is currently configured;
the displayed local hour can differ from the Mac's while UTC is correct.

## 3. Connect NetBird

It should start automatically. If Derek is not online in NetBird, run:

```bash
sudo netbird service start
sudo netbird up
netbird status
```

If it requests login, open the displayed link on the Mac and sign into
the same NetBird account already used for Derek. No reinstall or new
setup key is needed for a normal restart.

## 4. SSH from the Mac

Turn on NetBird on the Mac. Check that both devices appear online.
In the Mac terminal, run:

```bash
ssh -i ~/.ssh/derek_netbird_ed25519 -o HostKeyAlias=192.168.149.1 ubuntu@100.117.74.150
```

`100.117.74.150` is Derek's current NetBird IP. `HostKeyAlias` reuses its
saved SSH identity from the old hotspot address. The command uses the
existing key on this Mac.

To leave SSH:

```bash
exit
```

## If ROS commands wait for a subscriber afterward

NetBird being connected does not guarantee ROS discovery works. We observed
ROS starting on the hotspot address before eduroam connected, leaving its
network sockets bound to the old address. Restarting only `ros2 daemon`
did not fix those running nodes. This startup ordering still needs a
lasting fix.

After eduroam is connected, restarting the robot application can refresh
those sockets. It can also initialize the arm, so clear the arm's space
before running this on Derek:

```bash
sudo systemctl restart start_app_node.service
```

Wait about 20 seconds, then check:

```bash
ros2 daemon stop
ros2 daemon start
ros2 topic info /ros_robot_controller/bus_servo/set_position
```

Look for a subscriber before issuing a movement command. `publishing #1`
confirms publication; servo position feedback or observation confirms
physical movement.
