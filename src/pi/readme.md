### Config

`raspi-config`: enable UART serial, I2C

I2C:

https://www.circuitbasics.com/raspberry-pi-i2c-lcd-set-up-and-programming/

```bash
sudo apt-get install i2c-tools python3-smbus
```

device should now show up at `/dev/i2c-1`. Scan for screen with:

```bash
i2cdetect -y 1
```

encoder python test

```bash
sudo apt-get install pigpiod
sudo pip3 install pigpio
```

### Shutdown button

```bash
sudo apt-get install python3-pip
sudo pip3 install gpiozero
```

### System service setup

- Build tools

```bash
sudo apt-get install git make g++ gcc python3 udev
```

- Nodejs

    Example, see https://hassancorrigan.com/blog/install-nodejs-on-a-raspberry-pi-zero/

    ```bash
    cd /tmp
    wget https://unofficial-builds.nodejs.org/download/release/v14.16.1/node-v14.16.1-linux-armv6l.tar.xz
    tar xf node-v14.16.1-linux-armv6l.tar.xz
    sudo cp -R node-v14.16.1-linux-armv6l/* /usr/local
    which node npm
    ```

- Source

    ```bash
    mkdir -p ~/git && cd ~/git
    git clone git@bitbucket.org:owings1/moon-unit.git
    cd moon-unit
    npm install
    ```

- Service

    ```bash
    cd src/pi/service
    ./install.sh
    ```

### Docker setup

> NB: not enough memory on Pi Zero to run container.

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sh get-docker.sh

sudo usermod -aG docker pi

mkdir -p ~/docker/moon-unit
```

`IMAGE`:

```
owings1/moon-unit:latest-arm32
```

`run-container` (adjust device path):

```bash
#!/bin/bash

set -e

DIR=`dirname "$0"`
IMAGE=`cat "${DIR}/IMAGE"`
CONTAINER_NAME="moon-unit"
DEVICE1="/dev/ttyS0"

if [[ -z "$NO_PULL" ]]; then
  docker pull "$IMAGE"
else
  echo "NO_PULL set, Not pulling image"
fi

docker run -d --name="$CONTAINER_NAME" \
  --privileged \
  --restart always \
  --device "$DEVICE1" \
  -e "GAUGER_PORT=${DEVICE1}" \
  -e "GPIO_ENABLED=1" \
  -p 8080:8080 \
  "$IMAGE"
```
