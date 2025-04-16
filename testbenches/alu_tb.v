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
