For Raspberry Pi 5 

'''bash
sudo usermod -aG gpio $USER
newgrp gpio  # 立即激活组权限
'''

check if enabled new user-group

'''bash
groups | grep gpio
'''

'''bash
# 创建udev规则
echo 'KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"' | sudo tee /etc/udev/rules.d/99-gpio.rules

# 重载规则
sudo udevadm control --reload
sudo udevadm trigger
'''

** Notice: if you use raspi 5, explicitly use /dev/gpiochip4 to control gpoio **

for example:

'''python
import gpiod

gpiod.Chip('gpiochip4') as chip:  # 替换为chip4
        line = chip.get_line(17)           # 以BCM 17为例
        line.request(consumer='robotics', type=gpiod.LINE_REQ_DIR_OUT)
        line.set_value(1)
        print("GPIO控制成功！")
'''