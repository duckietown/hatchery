## Hatchery

An IDE for the Robot Operating System (runs on Mac OS X, Linux, Windows). Language support and code assistance for ROS applications. 

### Running

First, install Java (JDK 8 or higher). Then you will need to run:

`./gradlew runIde`

To open an existing ROS project inside the IDE, you can provide an absolute path to a ROS project like so:

`./gradlew runIde -Project="<absolute path to ROS project>"`

### Features 

Currently, Hatchery supports the following types of files:

* XML
* Python
* YAML
* CMake
* Bash

It also supports refactoring and navigation in XML files:

* roslaunch (`*.launch`, `*.test`)
* rospackage (`package.xml`)

### Planning

- [x] Implement preliminary project structure and XML support
- [x] Write an MVP/POC app that supports file renaming and refactoring.
- [ ] Add support for project templates and skeleton project creation.
- [x] Add support for deploying a project from the local machine to the remote.
- [ ] Add support for monitoring and tracking running code, viewing logs.
    - [ ] Save to local disk
    - [ ] Searching the log
- [ ] Collect crash dumps and link to the corresponding code points.
    - [ ] Link stack traces to source code
    - [ ] Copy environment info and crash dump to clipboard

### Overview

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