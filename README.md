## Project Overview
This repository contains a MIPS-like processor implementation in Verilog as part of the CS220 Computer Organization course.

## Author
Shlok Jain (230493) <br>
Akash Verma (220097) <br>
Team 10 <br>
Course: CS220 Computer Organization, Sem 24-25 even <br>
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

## Instruction Encoding

The processor uses a 32-bit instruction format with the following fields:

```
31      26 25    21 20    16 15    11 10     6 5       0
+--------+---------+--------+--------+--------+--------+
| opcode |    rd   |   rs   |   rt   | shamt  |  func  |  R-type
+--------+---------+--------+--------+--------+--------+

31      26 25    21 20    16 15                        0
+--------+---------+--------+------------------------+
| opcode |    rt   |   rs   |       immediate        |  I-type
+--------+---------+--------+------------------------+

31      26 25                                         0
+--------+--------------------------------------------+
| opcode |                   address                  |  J-type
+--------+--------------------------------------------+
```

### Common Opcodes
- **000000 (0)**: R-type instructions (functionality determined by func field)
- **000001 (1)**: addi (add immediate)
- **000111 (7)**: lw (load word)
- **001000 (8)**: sw (store word)
- **001001 (9)**: slti (set less than immediate)
- **001010 (10)**: seq (set equal)
- **010000-010101 (16-21)**: Branch instructions (beq, bne, bgt, bgte, ble, bleq)
- **011000-011010 (24-26)**: Jump instructions (j, jr, jal)
- **100000-101001 (32-41)**: Floating-point operations (mfc1, mtc1, add.s, sub.s, etc.)

### Function Codes (for R-type)
- **000000 (0)**: add
- **000001 (1)**: sub
- **000010 (2)**: and
- **000100 (4)**: or
- **001000 (8)**: slt (set less than)
- **001001 (9)**: sll (shift left logical)
- **001100 (12)**: mult (multiply)

The custom implementation includes support for both integer and floating-point operations with dedicated functional units.