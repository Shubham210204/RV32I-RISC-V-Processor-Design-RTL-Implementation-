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
