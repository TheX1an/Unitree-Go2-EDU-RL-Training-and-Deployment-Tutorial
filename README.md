# Unitree Go2 EDU: RL Training and Deployment Tutorial

This tutorial documents, in as much detail as possible, the author’s experience setting up the software environment, training reinforcement learning (RL) policies, and deploying them in both sim-to-sim and sim-to-real settings using the Unitree Go2 EDU quadruped robot. The deployment is carried out on both a personal computer and an onboard NVIDIA Jetson Orin Nano.

The purpose of this tutorial is twofold. First, it serves as a personal record of the RL training and deployment process on the Unitree Go2 EDU (hereafter referred to as Go2). Second, it aims to share practical insights with fellow beginners in the robotics community. Learning robotics independently can be challenging, even with a research-grade platform like Go2. It is my sincere hope that this tutorial provides helpful guidance to others navigating similar learning paths.

## System Requirements: Hardware and Software

To ensure the success of this tutorial, the hardware and software configurations currently used are listed below for reference. If your setup matches the specifications provided, I am confident you will be able to complete all the steps without major issues.

For those whose hardware or software differs, I wish you the best of luck. Based on my experience, many of the challenges related to software environment configuration and code compilation—particularly in the following sections—are closely tied to the specific hardware and software conditions of your system.

 - **Personal Computer:** Lenovo Legion Y7000P 2024
 - **CPU:** Intel Core i7-14650HX
 - **GPU:** NVIDIA GeForce RTX 4060 laptop
 - **RAM**: 64 GB (originally 16 GB, upgraded manually)
 - **Storage:** 1 TB SSD (original)
 - **Operating System:** Dual boot – Windows 11 (original) / Ubuntu 24.04.3 LTS
 - **Quadruped Robot:** Unitree Go2 EDU
 - **Onboard Computer:** NVIDIA Jetson Orin Nano 8GB (official expansion dock from Unitree)

## Installation of unitree_rl_lab for RL Policy Training in Go2 Locomotion

Personally, among various physics simulators such as Gazebo, MuJoCo, PyBullet, and Isaac Gym, I prefer the newly developed **NVIDIA Isaac Lab**—a GPU-based robot simulator—for studying reinforcement learning (RL)-based quadruped locomotion. Fortunately, Unitree, the company from which I acquired my Go2 EDU robot, has released an open-source project named **unitree_rl_lab** that leverages this platform for RL locomotion policy development. Therefore, this tutorial begins with the installation of `unitree_rl_lab`.

 1. **Installation of NVIDIA Isaac Sim**

	1.1 **System Requirements Verification**
	
 	The official GitHub page of **unitree_rl_lab**, contributed by Unitree, can be found [here](https://github.com/unitreerobotics/unitree_rl_lab). Although a complete installation guide is provided, several non-trivial issues may still arise.

	The first step in installing `unitree_rl_lab` is to install **Isaac Lab** on your laptop or desktop. In my case, all software was installed on a system running Ubuntu 24.04.3 LTS. The installation guide for Isaac Lab is available [here](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html).

	It is recommended to install Isaac Lab using `pip`. Before executing the commands provided in the official installation guide, make sure to review the [Isaac Sim system requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html#system-requirements), as Isaac Sim must be installed prior to Isaac Lab.

	If your hardware setup matches the **Basic hardware and software requirements** listed, the system requirements for installing Isaac Sim should generally be satisfied.
	
	**!!! Critical Notes !!!**  
	When using a dual-system setup (Windows 11 and Ubuntu 24.04) on a single laptop—as in my case—the installation of Ubuntu 24.04 and the proper NVIDIA GPU driver may require **switching from hybrid mode to discrete mode** in the BIOS settings. The BIOS can be accessed by pressing `F2` during system boot.

	Importantly, this mode setting is often the reason why Ubuntu fails to boot properly after installation in dual-system configurations. Hopefully, this note will be helpful for those encountering difficulties when installing Ubuntu alongside Windows.

	On my personal machine, **hybrid mode** was required during the installation of Ubuntu 24.04. However, after installing the NVIDIA GPU driver, the system needed to be switched to **discrete mode** in order to boot normally.
	
	1.2 **Installation of NVIDIA GPU Driver**

	Next, a proper NVIDIA GPU driver must be installed according to the official instructions provided [here](https://docs.omniverse.nvidia.com/dev-guide/latest/linux-troubleshooting.html), **before** installing Isaac Sim. Every step listed in the guide should be followed strictly. On my Ubuntu 24.04.3 LTS system, I installed the latest production branch driver, version **580.76.05**. 

	At the time of installation, my Ubuntu system was freshly installed and did not have any existing GPU drivers. This significantly simplified the installation process.

	**!!! Critical Notes !!!**  
	When installing the GPU driver, the default open-source driver on Ubuntu, **Nouveau**, must be disabled. Instructions for disabling Nouveau can be generated using ChatGPT.

	1.3 **Installation of Miniconda**

	It is recommended to install Isaac Sim and Isaac Lab within a virtual environment created using [Miniconda](https://docs.anaconda.com/miniconda/miniconda-other-installer-links/). The installation process is straightforward, but be sure to select the appropriate installer based on your system architecture (x86 or ARM).

	**!!! Critical Notes !!!**  
	When using commands such as `sudo apt`, `make`, or `cmake`, the currently activated virtual environment does **not** matter. However, when using `pip install`, it is essential to activate the correct Conda environment to ensure that all required libraries are installed in the intended environment.
	
	1.4 **Installation of Isaac Sim via pip**
    
	After completing Sections 1.1, 1.2, and 1.3, you can proceed to execute the commands provided in the [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html) to install Isaac Sim via the terminal. Ensure that the documentation version is set to **main**, as older versions are also available on the website. In this tutorial, Isaac Sim version **5.0.0** is used.

	After installation, follow the verification steps outlined in the documentation to confirm that Isaac Sim is properly installed. Note that the first startup may take several minutes—this is completely normal, so please be patient.

 2. **Installation of NVIDIA Isaac Lab**

	At this point, your Isaac Sim installation should hopefully be successful. You can continue following the instructions in the [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html) to complete the installation of Isaac Lab itself.

	If you are unsure when to activate the Conda environments created for Isaac Sim and Isaac Lab, a good rule of thumb is to **always activate the appropriate Conda environment** when installing either of these two software packages.

	First, clone the Isaac Lab repository from GitHub.

	**!!! Critical Notes !!!**  
	When cloning the Isaac Lab repository, using **HTTPS** is recommended. In my case, the SSH method failed (it got stuck during cloning, for reasons I do not understand). Therefore, I always choose the HTTPS option when performing `git clone`.

	After installation, you can follow the verification steps provided in the documentation to ensure Isaac Lab was installed correctly. The first startup may take several minutes—this is completely normal, so please be patient. A black window will appear after the wait, as shown in the documentation.

	The documentation also includes a section titled **Train a robot!**, which serves as an additional test of the installation. The `Isaac-Ant-v0` task can be executed using the default configuration and should complete within 10 minutes (on my machine). If the task runs successfully, congratulations—you have successfully installed Isaac Lab!

3. **Installation of unitree_rl_lab**

	After installing Isaac Lab, you can proceed to install **unitree_rl_lab**, provided by Unitree, by following the instructions [here](https://github.com/unitreerobotics/unitree_rl_lab?tab=readme-ov-file).

	**!!! Critical Notes !!!**  
	Cloning the repository and installing `unitree_rl_lab` using the provided instructions is straightforward. However, for the step involving **downloading Unitree USD files**, make sure to install **Git LFS** before cloning the `unitree_model` repository. Git LFS is required to download large files from remote repositories. Without it, critical robot model files will be missing, causing `unitree_rl_lab` to malfunction.

	Additionally, after cloning the `unitree_model` repository, set the `UNITREE_MODEL_DIR` environment variable to match your local directory path. Although the downloaded folder is originally named `unitree_model`, it must be **renamed to `unitree_usd`** as instructed.

	After installation, the official GitHub page provides instructions to verify the environment. In the `scripts` directory:
	- **`list_envs.py`** lists the available tasks in `unitree_rl_lab`
	- **`rsl_rl/train.py`** is used to train reinforcement learning policies
	- **`rsl_rl/play.py`** is used to run trained policies in the Isaac Lab simulator

## Sim-to-Sim Policy Deployment with unitree_mujoco

After training a reinforcement learning (RL) policy using `unitree_rl_lab`, it is recommended to transfer the policy to another simulation environment for **sim-to-sim testing** before deploying it to the real Go2 robot. 

For this purpose, Unitree has provided another open-source library called **unitree_mujoco**. This section outlines how to install and use `unitree_mujoco` to perform sim-to-sim testing.


 1. **Compilation of the Robot Controller**

	Before installing `unitree_mujoco` for sim-to-sim testing, you must first **compile the robot controller** by following the instructions on the [unitree_rl_lab GitHub page](https://github.com/unitreerobotics/unitree_rl_lab?tab=readme-ov-file). To compile the robot controller, `unitree_sdk2` must be installed as a prerequisite. The steps for installing `unitree_sdk2` and compiling the controller are also provided in the same GitHub repository.

	**!!! Critical Notes !!!**  
	Please note that the instructions provided by Unitree are originally intended for the **G1 humanoid robot**. However, for the purpose of this tutorial, the correct directory to use is:  
	**`unitree_rl_lab/deploy/robots/go2`**

	When compiling the Go2 controller, an additional required file—**`libonnxruntime.so`**—must be included for successful compilation. This file can be downloaded from the [official ONNX Runtime release page](https://github.com/microsoft/onnxruntime/releases).

	Be sure to download the appropriate version (**onnxruntime-1.22.0**) based on your system architecture and operating system. For example, on the author’s personal machine (x86_64 architecture running Linux), the file should be placed in:  
	**`unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib`**
	
2. **Installation of unitree_mujoco**

	 2.1 **Installation of MuJoCo**
    
	After the Go2 controller is successfully compiled (an executable named **`go2_ctrl`** will be generated in **`unitree_rl_lab/deploy/robots/go2/build`**), you can proceed to install **`unitree_mujoco`** to test the policy trained in the Isaac Lab simulator within the MuJoCo simulator—this is referred to as **sim-to-sim testing**.

	Installation instructions for `unitree_mujoco` are provided [here](https://github.com/unitreerobotics/unitree_mujoco?tab=readme-ov-file#installation).  
Make sure to install **the C++ simulator**, as it is required by the `unitree_rl_lab` deployment workflow.

	**!!! Critical Notes !!!**  
	- When compiling the Go2 controller, we previously installed **`unitree_sdk2`**. During the installation of `unitree_mujoco`, the instructions will prompt you to install it again.  
  	**Please follow the instructions and reinstall it if required—it will not cause issues.**

	- After installing `unitree_sdk2`, you will be required to install MuJoCo.  
  	**Important:** The version of MuJoCo **must be 3.2.7**.  
  	Therefore, **do not simply use `git clone`**—instead, download the source code directly from [here](https://github.com/google-deepmind/mujoco/releases).

	- **Be sure to download the source code**, as you’ll need to compile MuJoCo using `cmake` and `make`.

	- When compiling MuJoCo, the active Conda environment may default to **`base`**, where the Python version may be too high for MuJoCo 3.2.7.  
  	**To resolve this, the author installed Python 3.10**, which worked successfully.

	- During the installation of `unitree_mujoco`, a dependency named **`cyclonedds`** may also be required. You can download it [here](https://github.com/eclipse-cyclonedds/cyclonedds/releases).  
  	**Make sure to use version `0.10.2` only.**

	- When installing `cyclonedds` following the instructions [here](https://github.com/unitreerobotics/unitree_sdk2_python), an environment variable must be exported.  
  	**When exporting this variable, replace `~` with `$HOME` to avoid path resolution issues.**

	- Finally, note the Conda environment requirements:
  	- Use **`env_isaaclab`** when training the RL policy
  	- Use **`base`** when performing the sim-to-sim test
	 
	 2.2 **Compilation of unitree_mujoco**

	After successfully completing all the installation steps for MuJoCo, you can clone the `unitree_mujoco` repository from GitHub and compile it on your computer by following the instructions on the official page.

	**You can test the compilation by running the executable file `unitree_mujoco` located in**  
	**`/unitree_mujoco/simulate/build`**.

 3. **Sim-to-Sim Testing**
	
 	After successfully compiling the Go2 controller and installing `unitree_mujoco`, you are now ready to conduct the **sim-to-sim test**.
 
	3.1 **Configuration of the Simulation**

    Locate the `config.yaml` file in `/unitree_mujoco/simulate` and configure it as follows:

	```yaml
	robot: "go2"                        # Robot name: "go2", "b2", "b2w", "h1", "go2w", or "g1"
	robot_scene: "scene.xml"           # Robot scene path: /unitree_robots/[robot]/scene.xml
	domain_id: 0                       # Domain ID (0 for Go2, 1 for G1 humanoid)
	interface: "lo"                    # Network interface (e.g., "lo" for localhost)
	use_joystick: 1                    # Simulate Unitree Wireless Controller using a gamepad
	joystick_type: "switch"           # Gamepad layout: use "switch" even for Xbox controllers
	joystick_device: "/dev/input/js0" # Device path for the joystick
	joystick_bits: 16                 # Bit resolution (some controllers may only support 8-bit)
	print_scene_information: 1        # Print robot link, joint, and sensor information
	enable_elastic_band: 0            # Virtual spring band (used for humanoid robots like H1)
 	```
 
 	**!!! Critical Notes !!!**  
	- Available `robot_scene` files can be found in **`/unitree_mujoco/unitree_robots`**.  
	- The `domain_id` for **Go2** is `0`; for the **G1 humanoid**, it is `1`.  
	- The `interface` refers to the network card name. In most cases, `"lo"` (loopback) works correctly.  
	- Set `use_joystick` to `0` if no gamepad is available.  
	- **Surprisingly, common Xbox gamepads must be set as `"switch"` instead of `"xbox"` in `joystick_type`**. Although this may seem counterintuitive, failing to do so may prevent proper control of the simulated robot in the MuJoCo environment.  
	- The remaining parameters can typically be left as-is. Notably, `enable_elastic_band` is generally used only for humanoid robots (e.g., H1), which require suspension via an elastic band before booting.

	3.2 **Control of the Simulated Robot Using the Trained Policy**
    
	Following the official instructions, first launch the simulation, then run the robot controller to control the robot within the MuJoCo simulator.

	**!!! Critical Notes !!!**  
	- For Go2 testing, the controller executable is located in **`/unitree_rl_lab/deploy/robots/go2/build`**.  
	- To ensure the controller can correctly locate the trained policy, `train.py` and `play.py` in **`/unitree_rl_lab/scripts/rsl_rl`** must be executed exactly as instructed in the official guide.  
	  **Even the `cd` commands should be followed strictly**, as the trained and exported policies will only be saved in the correct directory under these conditions. The trained policy will be placed in a folder named **`logs`**, located at:  
	  **`/unitree_rl_lab/logs`**
	
	- If both the simulation and the controller are functioning properly, a standard Xbox gamepad can be used to control the simulated robot.
	
	- **When pressing `[L2 + A]`, press and hold `L2` first, then press `A` while holding `L2`. Avoid pressing too quickly—this is a simulation, not an action game.**
	
	- **As for the `[Start]` button, it typically refers to the one with three horizontal black lines, commonly used to open the menu.**

## Off-Board and Onboard Sim-to-Real Deployment

 1. **Off-Board Sim-to-Real**

	After completing the sim-to-sim test (from Isaac Lab to MuJoCo), the verified policy can be deployed to the real Go2 robot for a **sim-to-real test**. One common method is to run the RL-policy-based Go2 controller on an external personal computer and transmit control signals to the robot via a **network cable**.

	The official guideline for establishing communication between your personal computer and the Go2 robot can be found [here](https://support.unitree.com/home/en/developer/Quick_start).

	Once the network is properly configured, you can execute the Go2 controller on your personal computer using the following command:

	```bash
	./go2_ctrl --network xxxxxx
 	```
	Replace `xxxxxx` with your actual **network interface name**. For example, the author’s interface is `enp7s0`. You can check yours using the `ifconfig` command in the terminal.

	After running the Go2 controller, follow the on-screen instructions in the terminal to control the real Go2 robot.
	Note that the gamepad used for sim-to-sim testing should **not** be used here. Instead, use the **official Go2 wireless remote joystick** for real-world control.
	
 2. **Onboard Sim-to-Real**

	The offboard sim-to-real method discussed above requires a personal computer to run the Go2 controller and a network cable to transmit control signals, which makes outdoor field testing inconvenient. To address this limitation, Unitree officially provides **expansion docks** for high-performance onboard computing. With this hardware, it becomes possible to run the RL-policy-based Go2 controller **onboard**, fully leveraging the computational resources of the Go2 EDU robot.

	The most critical challenge in onboard sim-to-real deployment is compiling the Go2 controller **directly on the expansion dock** (in the author’s case, a **Jetson Orin Nano**) so that it can be executed locally on the robot.
	
	**!!! Critical Notes !!!**  
	The Go2 controller compiled on a personal computer cannot be directly transferred and executed on the onboard Jetson Orin Nano due to an architecture mismatch: the author’s PC is based on the `x86_64` architecture, whereas the Jetson Orin Nano uses `aarch64`. As a result, the architecture-dependent executable file is incompatible. To resolve this, the Go2 controller must be compiled **natively on the Jetson Orin Nano**.
	
	To begin, copy the entire `unitree_rl_lab` directory from the personal computer to the Jetson Orin Nano. This ensures that the exported RL policy (located in `/unitree_rl_lab/logs`) is also transferred, so that the newly compiled controller can locate and load it properly.

	**!!! Critical Notes !!!**  
	The Jetson Orin Nano provided with the Go2 EDU robot comes pre-installed with **Ubuntu 20.04**. To access its graphical user interface, you can use **NoMachine**, a remote desktop application specifically designed for remote system control. Fortunately, NoMachine is already installed on the Jetson Orin Nano by Unitree.

	To launch NoMachine, connect to the Jetson via SSH using the following command:

	```bash
	ssh unitree@192.168.123.18
	# Default password: 123
	```
	
 	Once logged in, run the following command to start the NoMachine service:

	```bash
	bash nomachine.sh
	```

 	Afterward, you will be able to control the Jetson Orin Nano remotely from your personal computer using the NoMachine client.

	As for transferring files to the Jetson, the author chose to use a USB flash drive for simplicity and reliability.

	After copying the `unitree_rl_lab` directory to the Jetson Orin Nano, the folder **`unitree_rl_lab/deploy/robots/go2/build`** should be deleted to allow for re-compilation. To ensure a successful build, several critical steps must be completed beforehand.

	**!!! Critical Notes !!!**  
	First, replace the directory **`/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0`** with the correct **aarch64** version of ONNX Runtime. You can download it from the official release page [here](https://github.com/microsoft/onnxruntime/releases).  
	Upon inspection, you will find that the contents of the x86_64 and aarch64 directories are nearly identical.
	
	**!!! Critical Notes !!!**  
	After replacing the ONNX Runtime files, modify the file **`/unitree_rl_lab/deploy/robots/go2/CMakeLists.txt`** to ensure that it correctly points to the new aarch64 ONNX Runtime directory.

	**!!! Critical Notes !!!**  
	Even after completing all the above steps, the compilation of the Go2 controller on the Jetson Orin Nano may still result in errors. After consulting ChatGPT, the following modifications were validated by the author to resolve the issue and enable successful compilation.
	
	**First**, add the following lines immediately **below the `project(...)` declaration** in  
	`/unitree_rl_lab/deploy/robots/go2/CMakeLists.txt`:
	
	```cmake
	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	set(CMAKE_CXX_EXTENSIONS OFF)
	```

	**Then**, execute the following commands in a terminal on the Jetson Orin Nano to insert missing header files required for successful compilation:

	```bash
	cd ~/unitree_rl_lab
	
	sed -i '1i #include <filesystem>' include/param.h
	sed -i '1i #include <filesystem>' include/FSM/State_RLBase.h
	sed -i '1i #include <algorithm>' include/isaaclab/manager/manager_term_cfg.h
	sed -i '1i #include <algorithm>' include/isaaclab/envs/mdp/actions/joint_actions.h
	sed -i '1i #include <algorithm>' include/isaaclab/envs/mdp/observations.h
	```

	These commands automatically insert the required `#include directives at the top of each specified file, resolving missing dependency issues during compilation.

	**Finally**, we can compile the Go2 controller directly on the Jetson Orin Nano using the following steps:

	```bash
	cd ~/unitree_rl_lab/deploy/robots/go2
	rm -rf build
	mkdir build && cd build
	cmake .. -DCMAKE_BUILD_TYPE=Release
	make -j$(nproc)
	```

 	This process will configure and build the Go2 controller in **Release** mode using all available CPU cores for maximum compilation efficiency.

	**For the author, even after completing all the steps above, a minor issue related to the `fmt` library remained.**  
	To resolve it, execute the following commands in a terminal on the Jetson Orin Nano:
	
	```bash
	sudo apt update
	sudo apt install -y libfmt-dev libspdlog-dev
	
	cd ~/unitree_rl_lab/deploy/robots/go2/build
	make -j$(nproc)
	```

 	These commands install the necessary development libraries (`libfmt and `libspdlog) and recompile the project to complete the build successfully.

	After all these efforts, the Go2 controller has been successfully compiled on the author's Go2 robot.  
The next step is to use **NoMachine** to access the Jetson Orin Nano and run the compiled RL-policy-based Go2 controller **onboard**.

---

If you have successfully followed this tutorial up to this point—**congratulations!** 🎉  
You have completed the full development pipeline for deploying a reinforcement learning locomotion policy on the Go2 robot:

1. **Trained** an RL locomotion policy using Isaac Lab  
2. **Performed** a sim-to-sim test using MuJoCo  
3. **Completed** an offboard sim-to-real test using a personal computer  
4. **Achieved** onboard sim-to-real deployment using the Jetson Orin Nano

This means you now have a complete working foundation for real-world RL-based quadruped locomotion.  
All future development and research can be built upon this.

**Have fun, and good luck with your next steps! 🚀**
