
module sap_tb();

reg clk,clr,run_prog;

SAP sap_dut(.in_clk(clk),.clr(clr),.run_prog(run_prog)
);

initial begin
	clk=0;
	run_prog = 1;
end
    
initial begin
    clr = 0;
#8  clr = 1; 
#15 clr = 0;
//#1050 $finish;
end

initial forever #10 clk=~clk;
	
endmodule

