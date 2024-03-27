# Lap Sim C++

This project aims to provide a comprehensive lap simulation tool written in C++, inspired by similar formula student projects or master thesis. The goal is to offer a robust and efficient simulation environment for analyzing vehicle performance on a track. This lap simulation tool allows for both static parameter setups and dynamic interactions, allowing for rapid prototyping and validation of vehicle configurations.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Dependencies](#dependencies)
- [Contributing](#contributing)
- [License](#license)

## Features

- Lap simulation with static parameters
- Lap simulation with dynamic interactions
- Modular codebase for easy modification and extension
- Output visualization for analysis
- Utilizes modern C++ features and best practices

## Installation

1. Clone this repository to your local machine:

    ```bash
    git clone https://github.com/GoncaloOlas/Lap-Sim-CPP.git
    ```

2. Navigate to the project directory:

    ```bash
    cd Lap-Sim-CPP
    ```

3. Compile the source code:

    ```bash
    make
    ```

## Usage

1. Customize the simulation parameters in the appropriate files (e.g., `constants.hpp`).

2. Choose the desired simulation mode in `main.cpp` (e.g., acceleration run, skid pad, track run).

3. Run the executable:

    ```bash
    ./TrackRun
    ```

4. Analyze the simulation results generated in the output files.

## Dependencies

- C++20 compiler (e.g., GCC, Clang)
- Make build system

Make sure to have these dependencies installed on your system before compiling the application.

## Contributing

Contributions are welcome! If you have any suggestions, improvements, or feature requests, feel free to open an issue or create a pull request.

## License

This project is licensed under the MIT license.
