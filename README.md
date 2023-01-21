# NRGRobot 2023

The robot code of [FRC Team 948 - Newport Robotics Group](www.nrg948.com) for FIRST Robotics Competition 2023 [CHARGED UP presented by Haas](https://youtu.be/0zpflsYc4PA) game.

## Setup

1. Install Git from https://git-scm.org.

2. Follow the instructions at [WPILib Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) to install the FRC development environment.

3. Clone the repository.

    * Windows

        ```powershell
        PS> git clone https://github.com/NRG948/NRGRobot2023.git
        ```

    * Linux/MacOS

        * Create an SSH key if you do not already have one.

            ```sh
            $ ssh-keygen -t rsa
            ```

            Press ENTER to accept the default location.

            Enter a passphrase and press ENTER, or simply press ENTER to use no passphrase.

        * Clone the repository.

            ```sh
            $ git clone git@github.com:NRG948/NRGRobot2023.git
            ```

## Build

To build the robot code, use the "WPILib: Build Robot Code" command from the ![WPILib](doc/image/wpilib-24.png) menu in Visual Studio Code, or run the following command from a command-line:

* Windows

    ```powershell
    PS> ./gradlew build -Dorg.gradle.java.home="C:\Users\Public\wpilib\2023\jdk" 
    ```

* Linux/MacOS

    ```sh
    $ ./gradlew build -Dorg.gradle.java.home="~/wpilib/2023/jdk" 
    ```

> **NOTE:** When using external library SNAPSHOTs, you may need to explicitly refresh the Gradle build cache to pickup changes. In that case, include the `--refresh-dependencies` option in in your build command.

## Deploy

To deploy the robot code, use the "WPILib: Deploy Robot Code" command from the ![WPILib](doc/image/wpilib-24.png) menu in Visual Studio Code, or run the following command from a command-line:

* Windows

    ```powershell
    PS> ./gradlew deploy -Dorg.gradle.java.home="C:\Users\Public\wpilib\2023\jdk" 
    ```

* Linux/MacOS

    ```sh
    $ ./gradlew deploy -Dorg.gradle.java.home="~/wpilib/2023/jdk" 
    ```
