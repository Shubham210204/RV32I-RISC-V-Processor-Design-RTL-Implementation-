module register_file_tb;
    reg clk, reg_write;
    reg [4:0] rs1, rs2, rd;
    reg [31:0] write_data;
    wire [31:0] read_data1, read_data2;

    register_file uut (.clk(clk), .reg_write(reg_write), .rs1(rs1), .rs2(rs2), .rd(rd), .write_data(write_data), .read_data1(read_data1), .read_data2(read_data2));

    initial begin
        $dumpfile("register_file.vcd");
        $dumpvars(0, register_file_tb);
    end

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        $monitor("rd=%d, wd=%h | rs1=%d, rs2=%d | r1=%h, r2=%h", rd, write_data, rs1, rs2, read_data1, read_data2);
        reg_write = 1; rd = 5'd1; write_data = 32'hDEADBEEF;
        #10 rs1 = 5'd1; rs2 = 5'd2;
        reg_write = 1; rd = 5'd2; write_data = 32'h12345678;
        #10 rs1 = 5'd1; rs2 = 5'd2;
        #10 $finish;
    end
endmodule
