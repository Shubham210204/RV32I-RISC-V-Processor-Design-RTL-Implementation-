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
