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
