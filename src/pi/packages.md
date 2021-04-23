## Raspbian packages

- Docker

    ```
    curl -fsSL https://get.docker.com -o get-docker.sh
    sh get-docker.sh
    sudo usermod -aG docker pi
    ```

- Build tools (usually noop)

    ```
    sudo apt-get install make g++ gcc python3 udev
    ```

- Nodejs

    Example, see https://hassancorrigan.com/blog/install-nodejs-on-a-raspberry-pi-zero/