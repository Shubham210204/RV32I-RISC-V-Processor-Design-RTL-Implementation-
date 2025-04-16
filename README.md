# **RV32I RISC-V Processor Design (RTL Implementation)**

## **Introduction**
This project focuses on the complete RTL design, simulation, and functional verification of a 32-bit RISC-V processor based on the RV32I instruction set architecture. It demonstrates a modular hardware design flow, starting from individual components to an integrated top-level processor module, using open-source tools and Verilog HDL.

The project aims to offer hands-on understanding of RISC-V architecture, instruction formats, datapath design, control unit functionality, and how all components work together in a pipelined architecture.

---

## **Contents**

1. [Tools Setup](#1-tools-setup)  
2. [About RISC-V](#2-about-risc-v)  
3. [RV32I Processor Design Flow](#3-rv32i-processor-design-flow)  
   - [3.1 Program Counter](#31-program-counter)  
   - [3.2 Instruction Memory](#32-instruction-memory)  
   - [3.3 Register File](#33-register-file)  
   - [3.4 Immediate Generator](#34-immediate-generator)  
   - [3.5 ALU](#35-alu)  
   - [3.6 ALU Control](#36-alu-control)  
   - [3.7 Main Control Unit](#37-main-control-unit)  
   - [3.8 Data Memory](#38-data-memory)  
   - [3.9 Top Module Integration](#39-top-module-integration)  
4. [Results & Discussion](#4-results--discussion)


## **1. Tools Setup**

### **1.1.1 VS Code**
**VS Code** is a **lightweight and powerful code editor** used for writing, debugging, and managing Verilog and VLSI design files.  
It offers **extensions for syntax highlighting, simulation, and debugging** to enhance the HDL design workflow.

### **Installation Steps**
```bash
sudo apt update
sudo apt install code
```

### **1.1.2 Icarus Verilog (Iverilog)**
**Icarus Verilog (Iverilog)** is an **open-source Verilog simulator** used for compiling and simulating digital designs.  
It is widely used in **VLSI design, FPGA development, and digital logic verification**.

### **Installation Steps**
```bash
git clone https://github.com/steveicarus/iverilog.git
cd iverilog
sh autoconf.sh
./configure
make
sudo make install
```

### **1.1.3 GTKWave**
**GTKWave** is an **open-source waveform viewer** used for analyzing **digital simulation outputs** from Verilog and VHDL simulations.  
It supports **VCD (Value Change Dump), LXT, LXT2, FST**, and other common waveform formats.

### **Installation Steps**
```bash
git clone https://github.com/gtkwave/gtkwave.git
cd gtkwave
./configure
make
sudo make install
```

📌 **Here ends the Tools and PDK setup** 

---

## **2. About RISC-V**

RISC-V is an open-source instruction set architecture (ISA) based on established reduced instruction set computer (RISC) principles. The RV32I base integer instruction set is a 32-bit architecture used for general-purpose computing.

### **Key Features:**
- Simple and modular design  
- Open and license-free  
- Extensible for custom applications  
- Instruction types: R, I, S, B, U, and J-type  
- Supports 32 general-purpose registers (x0 to x31)  

### **RV32I Base Instructions Include:**
- Arithmetic: `add`, `sub`, `addi`  
- Logical: `and`, `or`, `xor`, `andi`, `ori`, `xori`  
- Shift: `sll`, `srl`, `sra`  
- Load/Store: `lw`, `sw`  
- Branches: `beq`, `bne`, `blt`, `bge`  
- Jump: `jal`, `jalr`  

### **Pipeline & Datapath:**
The processor is built around a classic 5-stage pipeline:
- IF: Instruction Fetch  
- ID: Instruction Decode  
- EX: Execute  
- MEM: Memory Access  
- WB: Write Back  

---

## **3. RV32I Processor Design Flow**

Each module was designed individually using Verilog, tested via dedicated testbenches, and later integrated into a single top module.

### **3.1 Program Counter**
- Holds the address of the current instruction.  
- Increments by 4 to move to the next instruction.  

### **3.2 Instruction Memory**
- Stores 32-bit machine code.  
- Addressed using the PC value.  

### **3.3 Register File**
- Consists of 32 registers.  
- Supports simultaneous read from two registers and write to one.  

### **3.4 Immediate Generator**
- Extracts and sign-extends immediate values based on instruction format.  

### **3.5 ALU**
- Performs arithmetic and logical operations.  
- Controlled by ALU Control module.  

### **3.6 ALU Control**
- Decodes ALU operations using `funct3`, `funct7`, and ALUOp signals from control unit.  

### **3.7 Main Control Unit**
- Generates all necessary control signals based on opcode.  

### **3.8 Data Memory**
- Handles memory operations (load/store).  
- Accessed during MEM stage.  

### **3.9 Top Module Integration**
- Combines all modules.  
- Tested using a final testbench.  
- Simulated using Icarus Verilog and GTKWave.  

---

## **4. Results & Discussion**

- All individual modules were verified with custom testbenches.  
- Waveform verification confirmed correct data path operations.  
- Simulation results matched expected RISC-V instruction outputs.  
- The project gave insight into RTL design, pipelining, and ISA architecture.  

