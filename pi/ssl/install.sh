 #!/bin/bash
set -e
sudo mkdir -p /usr/share/ca-certificates/local
sudo cp *.crt /usr/share/ca-certificates/local/
sudo dpkg-reconfigure ca-certificates
