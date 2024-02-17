# RV32ISimulator
# Description

This project simulates an RV32I processor in Python, focusing on understanding the fundamental operations and stages of instruction processing. It includes implementations for instruction memory, data memory, register files, a decoder, a control unit, and an arithmetic logic unit (ALU), alongside single-stage and five-stage processing cores. This simulation is designed for educational purposes, showcasing the basic workings of a processor executing RV32I instructions.

# Features

Instruction Memory (InsMem) and Data Memory (DataMem) classes for handling memory operations.
RegisterFile class for simulating the register file of the processor.
Decoder class for decoding instructions into their respective types and operations.
ControlUnit class for determining the control signals based on instruction type.
ALU implementation for executing arithmetic and logic operations.
SingleStageCore and FiveStageCore classes for simulating processor cores with different pipeline stages.
Support for basic RV32I instructions and their execution in a simplified processor environment.
# Requirements

Python 3.x

# Installation

Clone or download this repository to your local machine.
Ensure Python 3.x is installed on your system.
Usage

To run the simulation, navigate to the directory containing the script and run the following command in the terminal:

shell
Copy code
python <script_name>.py --iodir <path_to_input_files>
<script_name>.py should be replaced with the name of the script file.
<path_to_input_files> should be the path to the directory containing the imem.txt and dmem.txt files, which represent the instruction memory and data memory, respectively.

# Input Files

imem.txt: Contains the binary representations of the instructions to be loaded into the instruction memory.
dmem.txt: Contains the initial state of the data memory.
Output

The simulation generates output files showing the state of the data memory and the register file after executing the instructions. It also prints performance metrics such as the total number of cycles taken, cycles per instruction (CPI), and instructions per cycle (IPC) for both single-stage and five-stage cores.

# Contributing

Contributions to this project are welcome. Please follow the standard process for contributing to projects on GitHub:

Fork the repository.
Create a new branch for each feature or improvement.
Submit a pull request from your branch to the main project.
