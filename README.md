# YOU MUST DOOOO!!!
## **BACKUP** 

- the ros2 workspace (main branch) before next new feature

- **save** all the motor config file!!!

## Before run, ensure:

**To ensure general users can access the serial port devices, permanently modify permissions for serial port devices (such as /dev/ttyUSB0, /dev/ttyACM0)**

### **(1) Create the Udev Rule File**

```bash
sudo vim /etc/udev/rules.d/99-tty.rules  
```

Add the following line:

```bash
SUBSYSTEM=="tty", GROUP="dialout", MODE="0666"  
```

### **(2) Reload Udev Rules**

```bash
sudo udevadm control --reload-rules  
sudo udevadm trigger  
```
### **(3) Reconnect the Device**

Unplug and replug your serial device (e.g., USB-to-serial adapter), then verify permissions:

```bash
ls -l /dev/ttyACM0  # or /dev/ttyUSB0  
```

Expected output:

```
crw-rw-rw- 1 root dialout 166, 0 Jan 1 12:00 /dev/ttyACM0  
```



*(Note: `rw-rw-rw-` means all users have read/write access.)*
