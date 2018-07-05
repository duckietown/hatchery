## Hatchery

An open source IDE for the [Robot Operating System](http://www.ros.org/).
Provides language support and code assistance for developing ROS applications, with additional tools for deploying and monitoring live applications.

### Installation

#### Prerequisites

<details> 
<summary><b>Java</b> must be installed prior to running Hatchery...</summary> 

Hatchery requires a JRE or JDK. First check you have one installed: `java -version`
 
[JDK 8](http://openjdk.java.net/install/) or higher is sufficient. Ubuntu/Debian: `sudo apt-get install openjdk-8-jdk`
</details>

<details> 
<summary><b>Duckietown participants</b> should configure their Duckietown environment as <a href="http://book.duckietown.org/">instructed</a>...</summary> 

Ensure `echo $DUCKIETOWN_ROOT` returns the correct path to your [Duckietown directory](https://github.com/duckietown/software).

If not, you should first run `source environment.sh` from inside the Duckietown software directory.

Hatchery will use `DUCKIETOWN_ROOT` as the default project directory, so you can omit the `-Project` flag below. 
</details>

#### Running

First, clone this repository and open the project directory using the command line.

`git clone https://github.com/breandan/hatchery && cd hatchery`

To launch the IDE (optionally, you can specify the path to an existing ROS project):

`./gradlew runIde [-Project="<ABSOLUTE_PATH_TO_ROS_PROJECT>"]`

On first launch, you may need to setup a Python SDK. From **File | Project Structure** (or alternately **Preferences | 🔍 Python Interpreter | Project Interpreter**), then select or create a new *Python SDK* to receive coding assistance in Python files.

#### Plugin

Existing users of JetBrains' IDEs (ex. PyCharm, CLion, or IntelliJ IDEA) can install Hatchery directly through the IDE, via **File | Settings | Plugins | Browse Repositories... | :mag: "hatchery"**. Older versions of Hatchery are also available through the [plugin repository](https://plugins.jetbrains.com/plugin/10290-hatchery). 

### Getting Started

Watch the following screencast for a demonstration of some features:

[![Screencast](https://img.youtube.com/vi/OU1_tqZs9EM/0.jpg)](https://www.youtube.com/watch?v=OU1_tqZs9EM)

### Features 

Currently, Hatchery supports the following [ROS filetypes](https://wiki.wpi.edu/robotics/ROS_File_Types):

- [x] ROS Launch (`*.launch`, `*.test`)
<!--
    -[x] Syntax highlighting
    -[x] Resource references (`$(find <directory>)...`)
-->
- [x] ROS Package (`package.xml`)
<!--
    -[x] Syntax highlighting
    -[x] Package references (`<build_depend>`, `<test_depend>`, `<run_depend>`)
-->
- [x] ROS URDF (`*.urdf.xacro`)
<!--
    -[x] Syntax highlighting
    -[x] Resource references (`$(find <directory>)...`)
-->
- [x] ROS Bag (`*.bag`)
<!--
    -[ ] Live logfile tracking
- [x] ROS Message (`*.msg`)
<!--
    -[x] Syntax highlighting
-->
- [x] ROS Service (`*.srv`)

It also supports refactoring and navigation in the following file types:

* XML
* Python
* YAML
* CMake
* Bash

### Planning

- [x] Implement preliminary project structure and XML support
- [x] Write an MVP/POC app that supports file renaming and refactoring
- [ ] Add support for project templates and skeleton project creation
- [x] Add support for deploying a project from the local machine to the remote
- [ ] Add support for monitoring and tracking running code, viewing logs
    - [ ] Save to local disk
    - [ ] Searching the log
- [ ] Collect crash dumps and link to the corresponding code points
    - [ ] Link stack traces to source code
    - [ ] Copy environment info and crash dump to clipboard
- [ ] Integration with the Robot Operating System (ROS)
    - [x] ROS 1 support ([Kinetic Kame](http://wiki.ros.org/kinetic) recommended)
    - [ ] [ROS 2](https://github.com/ros2/ros2/wiki) support
- [ ] Gazebo simulator integration
- [ ] C/C++ support with build automation
- [ ] Remote debugging and testing support
- [ ] Docker integration
    - [x] Basic Docker support
    - [ ] Remote host and script support
    - [ ] Docker hub namespace awareness
    - [ ] Support for platformio tooling
    - [ ] X11 forwarding and rqt support
- [ ] Static analysis
    - [x] Invalid dependency detection
    - [ ] Validate Python/msg/srv compatibility
    - [ ] ROS nodes and graph analysis via `rosdep`/`rqt_dep`
- [ ] [rqt](http://wiki.ros.org/rqt) plugin support
    - [ ] [`rqt_img_view`](http://wiki.ros.org/rqt_image_view) - View images
    - [ ] [`rqt_plot`](http://wiki.ros.org/rqt_plot) - Plot data visually
    - [ ] [`rqt_graph`](http://wiki.ros.org/rqt_graph) - Graph messages
    - [ ] [`rqt_dep`](http://wiki.ros.org/rqt_dep) - Visualize dependecies
    - [ ] [`rqt_bag`](http://wiki.ros.org/rqt_bag) - Replay bag files

### Roadmap

We are currently working to expand support for the following features:

* **Syntax support** - Highlighting, navigation, autocompletion
* **Program analysis** - Code inspections, intentions, and linting
* **Testing support** - Unit and integration testing, code coverage
* **Project creation** - Project setup and boilerplate code generation
* **Dependency management** - Track installed and missing packages
* **Monitoring utils** - Logging, diagnostics, profiling and visualization
* **Crash analytics** - Enhanced stack traces with source navigation
* **Build automation** - Delta rebuilds, cmake magic, code hotswap
* **ROS integration** - Nodes, topics, services, parameters, graphs
* **Duckumentation** - Usage instructions and supported features

### Authors

* [Breandan Considine](https://github.com/breandan)

### Special Thanks

* [Duckietown](https://duckietown.org)
* [Liam Paull](https://github.com/liampaull)
* [Open Robotics](https://www.openrobotics.org/)
