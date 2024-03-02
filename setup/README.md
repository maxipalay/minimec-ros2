# Minimec setup

This aims to provide some sort of documentation on how the minimec is setup.

Disclaimer: This setup process applies if you're using the exact same configuration, and even if so, you may run into issues. These instructions should get you most of the way there.

Others might still find this helpful as reference.

This is by no means a recommended example on how to configure a robot or network interface, but it works for this purpose.

## Setting up the raspberry pi

- Flash ubuntu server 22.04 onto a microSD card using Pi Imager. Configure username, password, ssh access, and I'd recommend you configure a network so you can ssh directly into the Pi upon boot. Note: In this case, the username is `msr` so you need to change the scripts to account for a change in username.
- Once the Pi is up and running and you can ssh into it, copy the setup folder to it - you can use `scp`. E.g. `scp -r setup <username>@<hostname>:/home/<username>/`
- `cd` into `setup/scripts`
- Add execution permissions to the included scripts: `chmod 744 *.sh`
- Set up the can interface on every boot  - `sudo ./config-can-service.sh`
- Install ROS2 Iron and necessary dependencies - `sudo ./install-ros.sh`. Note: this will take a while.
- I'd recommend you restart the Pi and check both network interfaces are configured correctly, and that the can interfce is up, you can check this using `ip add`.
- Create a workspace and copy the included ROS2 packages. Yout are pretty much setup to use this robot.

## About networking setup

In this configuration, I'm using the built in wireless card of the Pi to access a network with internet, and the ethernet interface is connected to a wireless router onboard which provides a WLAN that is meant to be shared between devices in the ROS network for this application.

The configuration of the wireless router is out of the scope of this setup, but it should be fairly simple for the current purpose.