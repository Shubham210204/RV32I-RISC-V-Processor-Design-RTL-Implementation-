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
