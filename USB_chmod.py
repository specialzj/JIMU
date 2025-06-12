import subprocess

def set_device_permissions():
    # 替换为您的 sudo 密码
    sudo_password = "123456"
    try:
        # 使用 echo 将密码传递给 sudo
        command = "sudo -S chmod 666 /dev/ttyACM1"
        process = subprocess.run(
            command,
            input=sudo_password + "\n",  # 将密码作为输入传递
            text=True,
            shell=True,
            check=True
        )
        command = "sudo -S chmod 666 /dev/ttyACM0"
        process = subprocess.run(
            command,
            input=sudo_password + "\n",  # 将密码作为输入传递
            text=True,
            shell=True,
            check=True
        )
        print("Permissions updated successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Failed to update permissions: {e}")

# 调用函数
# set_device_permissions()