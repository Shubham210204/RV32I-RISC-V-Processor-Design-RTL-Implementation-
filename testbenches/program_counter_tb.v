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
