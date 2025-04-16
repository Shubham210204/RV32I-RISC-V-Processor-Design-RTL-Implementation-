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
