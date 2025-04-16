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
