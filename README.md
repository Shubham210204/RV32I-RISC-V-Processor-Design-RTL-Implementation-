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

Verilog module for Program Counter:
```bash
module program_counter (
    input clk,
    input reset,
    input [31:0] pc_next,
    output reg [31:0] pc
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            pc <= 32'b0;
        else
            pc <= pc_next;
    end
endmodule
```
<br>Testbench for the above module:
```bash
module program_counter_tb;
    reg clk, reset;
    reg [31:0] pc_next;
    wire [31:0] pc;

    program_counter uut (.clk(clk), .reset(reset), .pc_next(pc_next), .pc(pc));

    always #5 clk = ~clk;

    initial begin
        $dumpfile("program_counter.vcd");
        $dumpvars(0, program_counter_tb);
    end

    initial begin
        $display("Time\tReset\tPC_Next\t\tPC");
        $monitor("%0d\t%b\t%h\t%h", $time, reset, pc_next, pc);
        clk = 0; reset = 1; pc_next = 32'h4;
        #10 reset = 0;
        #10 pc_next = 32'h8;
        #10 pc_next = 32'hC;
        #10 $finish;
    end
endmodule
```
<br>

### **3.2 Instruction Memory**
- Stores 32-bit machine code.  
- Addressed using the PC value.

Verilog module for Instruction Memory:
```bash
module instruction_memory (
    input [31:0] addr,
    output [31:0] instruction
);
    reg [31:0] memory [0:255]; // 256 instructions max

    initial begin
        // Load program here, e.g. memory[0] = 32'hXXXXXXXX;
        $readmemh("mem/program.hex", memory);
    end

    assign instruction = memory[addr[9:2]]; // word-aligned
endmodule
```
<br> Testbench for the above module:
```bash
module instruction_memory_tb;
    reg [31:0] addr;
    wire [31:0] instruction;

    instruction_memory uut (.addr(addr), .instruction(instruction));

    initial begin
        $dumpfile("instruction_memory.vcd");
        $dumpvars(0, instruction_memory_tb);
    end

    initial begin
        $readmemh("program.hex", uut.memory);
        $display("Addr\tInstruction");
        addr = 32'h0;
        #5 $display("%h\t%h", addr, instruction);
        addr = 32'h4;
        #5 $display("%h\t%h", addr, instruction);
        addr = 32'h8;
        #5 $display("%h\t%h", addr, instruction);
        #5 $finish;
    end
endmodule
```
<br>

### **3.3 Register File**
- Consists of 32 registers.  
- Supports simultaneous read from two registers and write to one.

Verilog module for Register File:
```bash
module register_file (
    input clk,
    input reg_write,
    input [4:0] rs1,
    input [4:0] rs2,
    input [4:0] rd,
    input [31:0] write_data,
    output [31:0] read_data1,
    output [31:0] read_data2
);
    reg [31:0] registers [0:31];

    assign read_data1 = registers[rs1];
    assign read_data2 = registers[rs2];

    always @(posedge clk) begin
        if (reg_write && rd != 5'b0)
            registers[rd] <= write_data;
    end
endmodule
```
<br>Testbench for the above module:
```bash
module register_file_tb;
    reg clk, reg_write;
    reg [4:0] rs1, rs2, rd;
    reg [31:0] write_data;
    wire [31:0] read_data1, read_data2;

    register_file uut (.clk(clk), .reg_write(reg_write), .rs1(rs1), .rs2(rs2), .rd(rd), .write_data(write_data), .read_data1(read_data1), .read_data2(read_data2));

    initial begin
        $dumpfile("register_file.vcd");
        $dumpvars(0, register_file_tb);
    end

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        $monitor("rd=%d, wd=%h | rs1=%d, rs2=%d | r1=%h, r2=%h", rd, write_data, rs1, rs2, read_data1, read_data2);
        reg_write = 1; rd = 5'd1; write_data = 32'hDEADBEEF;
        #10 rs1 = 5'd1; rs2 = 5'd2;
        reg_write = 1; rd = 5'd2; write_data = 32'h12345678;
        #10 rs1 = 5'd1; rs2 = 5'd2;
        #10 $finish;
    end
endmodule
```
<br>

### **3.4 Immediate Generator**
- Extracts and sign-extends immediate values based on instruction format.

Verilog module for Immediate Generator:
```bash
module imm_generator (
    input [31:0] instruction,
    output reg [31:0] imm_out
);
    wire [6:0] opcode = instruction[6:0];

    always @(*) begin
        case (opcode)
            7'b0000011, 7'b0010011: // I-type
                imm_out = {{20{instruction[31]}}, instruction[31:20]};
            7'b0100011: // S-type
                imm_out = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            7'b1100011: // B-type
                imm_out = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            7'b0110111, 7'b0010111: // U-type
                imm_out = {instruction[31:12], 12'b0};
            7'b1101111: // J-type
                imm_out = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};
            default:
                imm_out = 32'b0;
        endcase
    end
endmodule
```
<br>Testbench for the above module:
```bash
module imm_generator_tb;
    reg [31:0] instruction;
    wire [31:0] imm_out;

    imm_generator uut (.instruction(instruction), .imm_out(imm_out));

    initial begin
        $dumpfile("imm_generator.vcd");
        $dumpvars(0, imm_generator_tb);
    end

    initial begin
        $display("Instruction\t\tImmediate");
        instruction = 32'b000000000001_00000_000_00001_0000011; // lw x1, 1(x0)
        #5 $display("%b\t%h", instruction, imm_out);

        instruction = 32'b0000000_00001_00010_000_00011_0110011; // add x3, x2, x1
        #5 $display("%b\t%h", instruction, imm_out);

        instruction = 32'b0000000_00010_00001_000_00011_0100011; // sw x2, 3(x1)
        #5 $display("%b\t%h", instruction, imm_out);

        #10 $finish;
    end
endmodule
```
<br>

### **3.5 ALU**
- Performs arithmetic and logical operations.  
- Controlled by ALU Control module.

Verilog module for ALU:
```bash
module alu (
    input [31:0] a,
    input [31:0] b,
    input [3:0] alu_control,
    output reg [31:0] result,
    output zero
);
    always @(*) begin
        case (alu_control)
            4'b0000: result = a & b;
            4'b0001: result = a | b;
            4'b0010: result = a + b;
            4'b0110: result = a - b;
            4'b0111: result = (a < b) ? 1 : 0;
            4'b1100: result = ~(a | b);
            default: result = 0;
        endcase
    end

    assign zero = (result == 0);
endmodule
```
<br>Testbench for the above module:
```bash
module alu_tb;
    reg [31:0] a, b;
    reg [3:0] alu_control;
    wire [31:0] result;
    wire zero;

    alu uut (.a(a), .b(b), .alu_control(alu_control), .result(result), .zero(zero));
    
    initial begin
        $dumpfile("alu.vcd");
        $dumpvars(0, alu_tb);
    end

    initial begin
        $display("A\tB\tControl\tResult\tZero");
        a = 10; b = 5;

        alu_control = 4'b0010; #5 $display("%d\t%d\tADD\t%h\t%b", a, b, result, zero);
        alu_control = 4'b0110; #5 $display("%d\t%d\tSUB\t%h\t%b", a, b, result, zero);
        alu_control = 4'b0000; #5 $display("%d\t%d\tAND\t%h\t%b", a, b, result, zero);
        alu_control = 4'b0001; #5 $display("%d\t%d\tOR\t%h\t%b", a, b, result, zero);
        #10 $finish;
    end
endmodule
```
<br>

### **3.6 ALU Control**
- Decodes ALU operations using `funct3`, `funct7`, and ALUOp signals from control unit.

Verilof module for ALU Control:
```bash
module alu_control (
    input [1:0] alu_op,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg [3:0] alu_control
);
    always @(*) begin
        case (alu_op)
            2'b00: alu_control = 4'b0010; // Load/Store
            2'b01: alu_control = 4'b0110; // Branch (sub)
            2'b10: begin // R-type
                case ({funct7[5], funct3})
                    4'b0000: alu_control = 4'b0010; // ADD
                    4'b1000: alu_control = 4'b0110; // SUB
                    4'b0111: alu_control = 4'b0000; // AND
                    4'b0110: alu_control = 4'b0001; // OR
                    default: alu_control = 4'b1111;
                endcase
            end
            default: alu_control = 4'b1111;
        endcase
    end
endmodule
```
<br>Testbench for the above module:
```bash
module alu_control_tb;
    reg [1:0] alu_op;
    reg [2:0] funct3;
    reg [6:0] funct7;
    wire [3:0] alu_control;

    alu_control uut (.alu_op(alu_op), .funct3(funct3), .funct7(funct7), .alu_control(alu_control));

    initial begin
        $dumpfile("alu_control.vcd");
        $dumpvars(0, alu_control_tb);
    end

    initial begin
        alu_op = 2'b10; funct3 = 3'b000; funct7 = 7'b0000000; // ADD
        #5 $display("ADD Control = %b", alu_control);

        funct7 = 7'b0100000; // SUB
        #5 $display("SUB Control = %b", alu_control);

        funct3 = 3'b111; // AND
        #5 $display("AND Control = %b", alu_control);

        funct3 = 3'b110; // OR
        #5 $display("OR Control = %b", alu_control);

        alu_op = 2'b00; // LW/SW
        #5 $display("Load/Store Control = %b", alu_control);

        alu_op = 2'b01; // BEQ
        #5 $display("Branch Control = %b", alu_control);

        #10 $finish;
    end
endmodule
```
<br>

### **3.7 Main Control Unit**
- Generates all necessary control signals based on opcode.

Verilog module for Control Unit:
```bash
module control_unit (
    input [6:0] opcode,
    output reg alu_src,
    output reg mem_to_reg,
    output reg reg_write,
    output reg mem_read,
    output reg mem_write,
    output reg branch,
    output reg [1:0] alu_op
);
    always @(*) begin
        case (opcode)
            7'b0110011: begin // R-type
                alu_src = 0; mem_to_reg = 0; reg_write = 1;
                mem_read = 0; mem_write = 0; branch = 0;
                alu_op = 2'b10;
            end
            7'b0000011: begin // Load
                alu_src = 1; mem_to_reg = 1; reg_write = 1;
                mem_read = 1; mem_write = 0; branch = 0;
                alu_op = 2'b00;
            end
            7'b0100011: begin // Store
                alu_src = 1; mem_to_reg = 0; reg_write = 0;
                mem_read = 0; mem_write = 1; branch = 0;
                alu_op = 2'b00;
            end
            7'b1100011: begin // Branch
                alu_src = 0; mem_to_reg = 0; reg_write = 0;
                mem_read = 0; mem_write = 0; branch = 1;
                alu_op = 2'b01;
            end
            default: begin
                alu_src = 0; mem_to_reg = 0; reg_write = 0;
                mem_read = 0; mem_write = 0; branch = 0;
                alu_op = 2'b00;
            end
        endcase
    end
endmodule
```
<br>Testbench for the above module:
```bash
module control_unit_tb;
    reg [6:0] opcode;
    wire alu_src, mem_to_reg, reg_write, mem_read, mem_write, branch;
    wire [1:0] alu_op;

    control_unit uut (.opcode(opcode), .alu_src(alu_src), .mem_to_reg(mem_to_reg),
                      .reg_write(reg_write), .mem_read(mem_read),
                      .mem_write(mem_write), .branch(branch), .alu_op(alu_op));

    initial begin
        $dumpfile("control_unit.vcd");
        $dumpvars(0, control_unit_tb);
    end

    initial begin
        $display("Opcode\tALUSrc RegWrite MemRead MemWrite Branch ALUOp");
        opcode = 7'b0110011; #5 $display("R-type\t%b\t%b\t%b\t%b\t%b\t%2b", alu_src, reg_write, mem_read, mem_write, branch, alu_op);
        opcode = 7'b0000011; #5 $display("Load\t%b\t%b\t%b\t%b\t%b\t%2b", alu_src, reg_write, mem_read, mem_write, branch, alu_op);
        opcode = 7'b0100011; #5 $display("Store\t%b\t%b\t%b\t%b\t%b\t%2b", alu_src, reg_write, mem_read, mem_write, branch, alu_op);
        opcode = 7'b1100011; #5 $display("Branch\t%b\t%b\t%b\t%b\t%b\t%2b", alu_src, reg_write, mem_read, mem_write, branch, alu_op);
        #10 $finish;
    end
endmodule
```
<br>

### **3.8 Data Memory**
- Handles memory operations (load/store).  
- Accessed during MEM stage.

Verilog module for Data Memory:
```bash
module data_memory (
    input clk,
    input mem_read,
    input mem_write,
    input [31:0] addr,
    input [31:0] write_data,
    output [31:0] read_data
);
    reg [31:0] memory [0:255];

    assign read_data = (mem_read) ? memory[addr[9:2]] : 32'b0;

    always @(posedge clk) begin
        if (mem_write)
            memory[addr[9:2]] <= write_data;
    end
endmodule
```
<br>Testbench for the above module:
```bash
module data_memory_tb;
    reg clk, mem_read, mem_write;
    reg [31:0] addr, write_data;
    wire [31:0] read_data;

    data_memory uut (.clk(clk), .mem_read(mem_read), .mem_write(mem_write),
                     .addr(addr), .write_data(write_data), .read_data(read_data));

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        $dumpfile("data_memory.vcd");
        $dumpvars(0, data_memory_tb);
    end

    initial begin
        mem_read = 0; mem_write = 1;
        addr = 32'h0; write_data = 32'hCAFEBABE;
        #10 mem_write = 0; mem_read = 1;
        #10 $display("Read from memory[0] = %h", read_data);
        #10 $finish;
    end
endmodule
```
<br>

### **3.9 Top Module Integration**
- Combines all modules.
```bash
module riscv_top(
    input clk,
    input reset,
    output [31:0] debug_pc,  // <-- ADD THIS
    output [31:0] debug_inst // <-- ADD THIS
);

    // Wires
    wire [31:0] pc_current, pc_next, instruction;
    wire [6:0] opcode;
    wire [4:0] rs1, rs2, rd;
    wire [2:0] funct3;
    wire [6:0] funct7;
    wire [31:0] reg_data1, reg_data2, imm, alu_input2, alu_result;
    wire [3:0] alu_control;
    wire [31:0] mem_data;
    wire alu_src, mem_to_reg, reg_write, mem_read, mem_write, branch;
    wire [1:0] alu_op;
    wire zero;
    wire [31:0] write_back_data;

    assign debug_pc = pc_current;
    assign debug_inst = instruction;

    // PC Register
    program_counter pc_reg (
        .clk(clk), .reset(reset), .pc_next(pc_next), .pc(pc_current)
    );

    // Instruction Memory
    instruction_memory imem (
        .addr(pc_current), .instruction(instruction)
    );

    assign opcode = instruction[6:0];
    assign rd     = instruction[11:7];
    assign funct3 = instruction[14:12];
    assign rs1    = instruction[19:15];
    assign rs2    = instruction[24:20];
    assign funct7 = instruction[31:25];

    // Control Unit
    control_unit cu (
        .opcode(opcode),
        .alu_src(alu_src), .mem_to_reg(mem_to_reg), .reg_write(reg_write),
        .mem_read(mem_read), .mem_write(mem_write), .branch(branch),
        .alu_op(alu_op)
    );

    // Register File
    register_file rf (
        .clk(clk), .reg_write(reg_write), .rs1(rs1), .rs2(rs2), .rd(rd),
        .write_data(write_back_data), .read_data1(reg_data1), .read_data2(reg_data2)
    );

    // Immediate Generator
    imm_generator ig (
        .instruction(instruction), .imm_out(imm)
    );

    // ALU Control
    alu_control ac (
        .alu_op(alu_op), .funct3(funct3), .funct7(funct7), .alu_control(alu_control)
    );

    // MUX for ALU input
    assign alu_input2 = (alu_src) ? imm : reg_data2;

    // ALU
    alu alu_core (
        .a(reg_data1), .b(alu_input2), .alu_control(alu_control),
        .result(alu_result), .zero(zero)
    );
 
    // Data Memory
    data_memory dmem (
        .clk(clk), .mem_read(mem_read), .mem_write(mem_write),
        .addr(alu_result), .write_data(reg_data2), .read_data(mem_data)
    );

    // Write Back
    assign write_back_data = (mem_to_reg) ? mem_data : alu_result;

    // PC Update
    assign pc_next = (branch && zero) ? pc_current + imm : pc_current + 4;

endmodule
```
<br>
- Tested using a final testbench.
```bash
module riscv_top_tb;
    reg clk, reset;

    riscv_top uut (
        .clk(clk),
        .reset(reset)
    );

    initial begin
        $dumpfile("riscv_top.vcd");
        $dumpvars(0);
    end


    initial begin
        $dumpfile("riscv_top.vcd");
        $dumpvars(0, riscv_top_tb);

        clk = 0;
        reset = 1;
        #10 reset = 0;

        // Let it run for some cycles
        #200 $finish;
    end

    always #5 clk = ~clk; // 10ns clock period
endmodule
```
<br>
- Simulated using Icarus Verilog and GTKWave.  

---

## **4. Results & Discussion**

- All individual modules were verified with custom testbenches.  
- Waveform verification confirmed correct data path operations.  
- Simulation results matched expected RISC-V instruction outputs.  
- The project gave insight into RTL design, pipelining, and ISA architecture.  

