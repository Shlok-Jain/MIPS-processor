## Project Overview
This repository contains a MIPS-like processor implementation in Verilog as part of the CS220 Computer Organization course.

## Author
Shlok Jain (230493)
Indian Institute of Technology, Kanpur

## Project Structure
```
├── assignment_8.pdf        # Assignment details
├── instructions_encoding.txt # Sample instruction encodings
├── processor.v             # Verilog implementation of the processor
└── README.md               # This file
```

## Processor Architecture
This is a MIPS-like processor designed to execute a subset of MIPS instructions. The architecture is based on a single-cycle design, where each instruction is executed in one clock cycle.

### Core Components
- **CPU**: The main processor module that connects all components
- **RegisterFile**: 32 general-purpose registers for integer operations
- **ALU**: Arithmetic and Logic Unit for integer operations
- **FPU**: Floating Point Unit for floating point operations
- **Memory**: Distributed memory for instructions and data
- **PC Controller**: Program counter management for instruction sequencing
- **Controller**: Main control unit for instruction decoding and control signal generation

### Memory System
- **Instruction Memory**: 1024 x 32-bit memory for storing program instructions
- **Data Memory**: 1024 x 32-bit memory for storing data
- **Register File**: 32 x 32-bit registers for general-purpose storage
- **Floating Point Registers**: 32 x 32-bit registers for floating point operations

## Instruction Set
The processor implements a custom instruction set with:
- R-type instructions (register-register operations)
- I-type instructions (immediate operations)
- Memory operations (load/store)
- Branch instructions
- Jump instructions
- Floating point instructions

## Testbench
The included testbench demonstrates an insertion sort implementation using the processor instructions.