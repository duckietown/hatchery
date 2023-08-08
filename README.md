## 🐣 Hatchery

[![][teamcity-status-svg]][teamcity-build-status]
[![][plugin-repo-svg]][plugin-repo-page]
[![][plugin-download-svg]][plugin-repo-page]

**Note**: This project is currently unmaintained and seeking a maintainer. We will try to keep it running on the latest version of the IntelliJ Platform, but no guarantees. If you are interested in contributing, please contact [Duckietown](https://duckietown.org).

An open source IDE for the [Robot Operating System](http://www.ros.org/).

Provides language support and code assistance for developing ROS applications, with tools for deploying and monitoring live applications.

### Installation

#### Prerequisites

<details>
<summary><b>Java</b> must be installed prior to building Hatchery...</summary>

Building Hatchery requires a JRE or JDK. First check you have one installed: `java -version`
 
[JDK 8](http://openjdk.java.net/install/) or higher is sufficient. Ubuntu/Debian: 
```bash
sudo add-apt-repository ppa:openjdk-r/ppa
sudo apt-get update
sudo apt-get install openjdk-8-jdk
```

First, clone this repository and open the project directory using the command line.

`git clone https://github.com/breandan/hatchery && cd hatchery`

To launch the IDE (optionally, you can specify the path to an existing ROS project):

`./gradlew runIde [-Project="<ABSOLUTE_PATH_TO_ROS_PROJECT>"]`

On first launch, you may need to setup a Python SDK. From **File | Project Structure** (or alternately **Preferences | :mag: Python Interpreter | Project Interpreter**), then select or create a new *Python SDK* to receive coding assistance in Python files.
</details>

<details>
<summary><b>CLion users</b> should follow the <a href="https://www.jetbrains.com/help/clion/2019.3/ros-setup-tutorial.html">ROS Setup Tutorial</a> to configure build paths...</summary> 

and access CLion-specific features such as <a href="https://www.jetbrains.com/help/clion/2019.3/ros-setup-tutorial.html#43578262">linking Catkin libraries</a>, <a href="https://www.jetbrains.com/help/clion/2019.3/ros-setup-tutorial.html#80196d29">running a ROS node from the IDE</a> and <a href="https://www.jetbrains.com/help/clion/2019.3/ros-setup-tutorial.html#931260ab">attaching a debugger to a running node</a>. The Hatchery plugin can be <a href="https://www.jetbrains.com/help/idea/managing-plugins.html#install">installed</a> in the usual way from the settings menu.
</details>

<details>
<summary><b>Duckietown participants</b> should configure their Duckietown environment as <a href="http://book.duckietown.org/">instructed</a>...</summary>

Ensure `echo $DUCKIETOWN_ROOT` returns the correct path to your [Duckietown directory](https://github.com/duckietown/software).

If not, you should first run `source environment.sh` from inside the Duckietown software directory.

Hatchery will use `DUCKIETOWN_ROOT` as the default project directory, so you can omit the `-Project` flag below.
</details>

Existing users of JetBrains' IDEs (ex. PyCharm, CLion, or IntelliJ IDEA) can install Hatchery directly through the IDE, via **File | Settings | Plugins | Marketplace... | :mag: "hatchery"**. Older versions of Hatchery are also available through the [plugin repository](https://plugins.jetbrains.com/plugin/10290-hatchery).

For the adventurous, untested, canary builds are available on our [TeamCity build server](https://teamcity.jetbrains.com/repository/download/hatchery_buildplugin/.lastSuccessful/hatchery.zip?guest=1). Download the plugin ZIP file and [install it from disk](https://www.jetbrains.com/help/idea/managing-plugins.html#installing-plugins-from-disk).

### Contributing

To contribute to this project, run the following command from inside the project root directory:

`./gradlew runIde -PluginDev`

This will download and run IntelliJ IDEA CE and open the project. You are ready to get started!

#### ROS Enrivonment

Hatchery tries to locate the ROS installation directory, but it helps if you are running the IDE inside with the ROS environment. In most cases, we can detect the ROS installation path, however you may need to manually configure the [ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) beforehand.

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
- [x] Add support for project templates and skeleton project creation
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
- [x] Gazebo simulator integration
- [ ] C/C++ support with build automation
- [ ] Remote debugging and testing support
- [ ] Docker integration
    - [x] Basic Docker support
    - [ ] Remote host and script support
    - [ ] Docker Hub namespace awareness
    - [ ] Support for platformio tooling
    - [ ] X11 forwarding and rqt support
- [ ] Static analysis
    - [x] Invalid dependency detection
    - [ ] Validate Python/msg/srv compatibility
    - [ ] ROS nodes and graph analysis via `rosdep`/`rqt_dep`
- [ ] [rqt](http://wiki.ros.org/rqt) plugin support
    - [x] [`rqt_img_view`](http://wiki.ros.org/rqt_image_view) - View images
    - [x] [`rqt_plot`](http://wiki.ros.org/rqt_plot) - Plot data visually
    - [x] [`rqt_graph`](http://wiki.ros.org/rqt_graph) - Graph messages
    - [x] [`rqt_dep`](http://wiki.ros.org/rqt_dep) - Visualize dependencies
    - [x] [`rqt_bag`](http://wiki.ros.org/rqt_bag) - Replay and edit bag files
    - [ ] [rqt_common](http://wiki.ros.org/rqt_common_plugins) - Other common plugins

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

* [Université de Montréal](https://en.diro.umontreal.ca/home/) - For supplying hardware and research facilities.
* [Duckietown](https://duckietown.org) - An open, inexpensive and flexible platform for autonomy education and research.
* [Liam Paull](https://github.com/liampaull) - For academic advice and guidance.
* [Michalis Famelis](https://michalis.famelis.info/) - For academic advice and guidance.
* [Rusi Hristov](https://github.com/rusi) - For technical advice and guidance.
* [Paolo Achdjian](https://github.com/paoloach) - For contributing code from a [ROS-plugin](https://github.com/paoloach/ROS-JetBrains-Plugin).
* [nuTonomy](https://www.nutonomy.com/) - For sponsoring development on the AI-DO 2018 competition.
* [JetBrains](https://www.jetbrains.com/) - For [collaboration](https://research.jetbrains.org/duckietown) and [TeamCity](https://www.jetbrains.com/teamcity/) build services.
* [Open Robotics](https://www.openrobotics.org/) - For [ROS](https://www.ros.org) development and technical support.

<!-- Badges -->
[teamcity-build-status]: https://teamcity.jetbrains.com/viewType.html?buildTypeId=hatchery_buildplugin&guest=1
[teamcity-status-svg]: https://teamcity.jetbrains.com/app/rest/builds/buildType:hatchery_buildplugin/statusIcon.svg
[plugin-repo-page]: https://plugins.jetbrains.com/plugin/10290-hatchery
[plugin-repo-svg]: https://img.shields.io/jetbrains/plugin/v/10290-hatchery.svg
[plugin-download-svg]: https://img.shields.io/jetbrains/plugin/d/10290-hatchery.svg
