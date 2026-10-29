# Project diagrams

Visual resources I use to explain how this project is put together. Each diagram comes as an `.svg` (editable, sharp at any size) and a `.png` (for quick viewing or slides).

## 01 · Docker on my Mac and on GitHub

![Docker on my Mac and on GitHub](01_docker_mac_vs_github.png)

- **Grey — the physical computer.** My Mac, or GitHub's servers in the cloud.
- **Green — the Linux virtual machine.** There is one on each side and they are independent: on my Mac it is created by Docker Desktop (macOS is not Linux); on GitHub it is created by GitHub (the Actions *runner* is already an Ubuntu VM).
- **Purple — the container.** It is exactly the same on both sides (`warehouse-nav2:jazzy`, with ROS 2, Nav2, Gazebo and my test suite). That is why whatever runs on my Mac runs the same way in GitHub Actions.
