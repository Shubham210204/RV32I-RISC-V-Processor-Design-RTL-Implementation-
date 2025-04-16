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
