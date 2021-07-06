#curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s - --no-modify-path -y --default-toolchain stable

sudo apt update
sudo apt install -y build-essential libxcb-render0 libxcb-shape0 libxcb-xfixes0
sudo apt dist-upgrade