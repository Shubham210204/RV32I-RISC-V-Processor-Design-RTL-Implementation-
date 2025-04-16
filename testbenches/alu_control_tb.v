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
