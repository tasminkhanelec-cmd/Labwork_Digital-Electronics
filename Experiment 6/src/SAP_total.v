module pc( Cp, nCLK, nCLR, Ep, WBUSlower);
input Cp;
input nCLK;
input nCLR;
input Ep;
output[3:0] WBUSlower;

reg[3:0] nextPC;
reg[3:0] WBUSlower;

always @(negedge nCLK) // negative edge of nCLK means positive edge of CLK
	begin
		if( Cp == 1)
			begin
				nextPC = nextPC + 1;
			end
		if ( nCLR == 0)
			/* Reset Program Counter */
			begin
				nextPC = 4'b0000;
				//WBUSlower = 4'b0000;
			end
	end
/*
* Keep WBUS in high impedence state when Ep is low (during this time some other module is using
* When Ep is high, output the contents of PC to WBUS
*/
always @(*) WBUSlower = (Ep) ? nextPC : 4'bzzzz;
endmodule



module inputMAR(nLm, clk, in, ram_addr, ram_nrd, ram_nwr, run_prog);
input clk, nLm, run_prog; //run_prog = 1 means in run mode not in programming mode
output ram_nrd, ram_nwr;
reg ram_nrd, ram_nwr;
parameter wordsize = 4;
input [3:0] in;
output [3:0] ram_addr;
reg [3:0] ram_addr;
always @ (posedge clk)
begin
if(run_prog) begin
	{ram_nrd, ram_nwr} = 2'b01;//make nrd enabled and nwr disabled b/c in run mode ram become ROM
	if(!nLm) ram_addr = in;
	else ram_addr <= ram_addr;
end
else begin
{ram_nrd, ram_nwr} = 2'b10;
if(!nLm) ram_addr = in; //make nrd enabled and nwr disabled b/c in run mode
else ram_addr <= ram_addr;
end
end
endmodule

// ram ram_sap1(.clk(clk), .nrd(ram_nrd), .nwr(ram_nwr), .nce(nCee), .addr(ram_addr), .data(WBUS));
module ram(clk, nrd, nwr, nce, addr, data);
input clk, nrd, nwr, nce;
output [7:0] data;
input [3:0] addr;
reg [7:0] rammem[0:2**4];
reg [7:0] data;

initial begin
		$readmemh("ram.mem",rammem);
end

always @ (nce or nwr or nrd) //asynchronous RAM
begin
if(!nrd)  //nrd is always zero, always read from ram.
	data <= !nce? rammem[addr] : 8'bz;
else if(!nwr)
	rammem[addr] <= !nce? data:rammem[addr];
end
endmodule

//IR IR_sap1(.clk(clk), .nli(nLi), .nei(nEi), .clr(clr), .wbus(WBUS), .wbuslower(WBUS[3:0]), .wbusup(opcode)); //
module IR(clk, nli, nei, clr, wbus, wbuslower, wbusup);
input clk, nli, nei, clr;
input [7:0] wbus;
output [3:0] wbuslower, wbusup;
reg [7:0]i_reg;
reg [3:0] wbuslower;
assign wbusup = i_reg[7:4];
always @ (posedge clk)
begin
	if (clr)
		begin i_reg = 8'b0 ; wbuslower = 4'bz; end
	else if(!nli) 
		begin
			i_reg = wbus; wbuslower = 4'bz; 
		end
end

always @(nei)
	if (!nei) 
	wbuslower = i_reg[3:0];
	else wbuslower = 4'bz;
endmodule


//accumulator ACC_sap1(.clk(clk), .nLa(nLa), .Ea(Ea), .accout(WBUS), .accin(WBUS), .accreg(ACC_to_ALU));
module accumulator(clk, nLa, Ea, accout, accin, accreg);
input clk, nLa, Ea;
input [7:0] accin;
output [7:0] accout, accreg;
reg [7:0] accout, accreg;
always @(posedge clk or posedge Ea)
	begin
		if(!nLa) //nLa = 1, Ea = 0
			begin
			accreg <= accin; 
			//accout <= 8'bz;
			end
		if(Ea)
		accout <= accreg;
		else accout <= 8'bz;
	end
endmodule

//addsub addsub_sap1(.Su(Su), .Eu(Eu), .ina(ACC_to_ALU), .inb(regB_to_ALU), .result(WBUS));
module addsub(Su, Eu, ina, inb, result);
input Su, Eu;
parameter wordsize = 8;
input [7:0] ina, inb;
output reg [7:0] result;
wire [7:0] ina, inb;
always @(*)
	if(Eu)
		result <= Su? ina-inb:ina+inb;
	else result <= 8'bz;
endmodule


module regB(nLb, clk, in, out);
input clk, nLb;
parameter wordsize = 8;
input [7:0] in;
output [7:0] out;
reg [7:0] out;
always @ (posedge clk)
begin
if(!nLb) out = in;
end
endmodule

//outreg outreg_sap1(.nLo(nLo), .clk(clk), .in(WBUS), .out(result));
module outreg(nLo, clk, in, out);
input clk, nLo;
input [7:0] in;
output [7:0] out;
reg [7:0] out;
always @ (posedge clk)
begin
if(!nLo) out = in;
end
endmodule


module control_unit(clk, clr, cntrl_bus, clk_hlt, opcode);
input clk,clr;
output clk_hlt;
parameter opcodesize = 4;
input [opcodesize-1:0] opcode;
parameter cntrlbussize = 12;
parameter T1 = 6'b000001,
T2 = 6'b000010,
T3 = 6'b000100,
T4 = 6'b001000,
T5 = 6'b010000,
T6 = 6'b100000;
output [cntrlbussize-1:0] cntrl_bus;
// ring counter
reg [5:0] ringcount;
reg [cntrlbussize-1:0] cntrl_bus;
wire [5:0]state ;
reg clk_hlt;

always @ (negedge clk)
	begin
		if (clr)
		ringcount = 6'b000001;
		else case(ringcount)
			T1: ringcount <= T2;
			T2: ringcount <= T3;
			T3: ringcount <= T4;
			T4: ringcount <= T5;
			T5: ringcount <= T6;
			T6: ringcount <= T1;
		endcase
end
//end of ring counter
/*Mealy state machine in which next state is missing in a sense that
ringcounter indepedently calculate determine it*/
assign state = ringcount;
always @(ringcount)
	begin	
		case (state)
			T1: begin cntrl_bus = 12'h5E3; clk_hlt = 0; end//fetch cycle start here PC -> MAR
			T2: cntrl_bus = 12'hBE3;
			T3: cntrl_bus = 12'h263; //fetch cycle end here
		endcase
		case ({state, opcode})
			//LDA operation
			{T4, 4'h0}: cntrl_bus = 12'h1A3;
			{T5, 4'h0}: cntrl_bus = 12'h2C3;
			{T6, 4'h0}: cntrl_bus = 12'h3E3;
			//ADD
			{T4, 4'h1}: cntrl_bus = 12'h1A3;
			{T5, 4'h1}: cntrl_bus = 12'h2E1;
			{T6, 4'h1}: cntrl_bus = 12'h3C7;
			//SUB
			{T4, 4'h2}: cntrl_bus = 12'h1A3;
			{T5, 4'h2}: cntrl_bus = 12'h2E1;
			{T6, 4'h2}: cntrl_bus = 12'h3CF;
			//OUT
			{T4, 4'hE}: cntrl_bus = 12'h3F2;
			{T5, 4'hE}: cntrl_bus = 12'h3E3;
			{T6, 4'hE}: cntrl_bus = 12'h3E3;
			//HLT
			{T4, 4'hF}: clk_hlt = 1;
			endcase
	end
endmodule

//module SAP(in_clk, clr, result, run_prog, ram_sel, Wbus_ext, nLm_ext); 
module SAP(in_clk, clr, result, run_prog, ram_sel, nLm_ext); 


//ram_nwr and nCe is placed in portlist because manually programming will be done
input in_clk, clr, run_prog, ram_sel, nLm_ext;
wire clk, clk_hlt;

// Assinging clk with the clk_hlt logic
assign clk = in_clk & (!clk_hlt);


//varibles for run and program mode of SAP1 in FPGA it is connected to the switches
wire ram_nrd, ram_nwr, run_prog, nLm_int;

// Wires for the Bus
wire [7:0] WBUS;
//input [3:0] Wbus_ext;



// Defining all the Control Signals
wire Cp, Ep, nLm, nCe, nLi, nEi, nLa, Ea, Su, Eu, nLb, nLo;
wire [11:0] cntrl_bus;
assign {Cp, Ep, nLm, nCe, nLi, nEi, nLa, Ea, Su, Eu, nLb, nLo} = cntrl_bus;

// Wires related to the connection between ALU and ACC,regB
wire [7:0] ACC_to_ALU, regB_to_ALU;

// Wires for the RAM addr and Opcode
wire [3:0] ram_addr, opcode;

// Wires for output Reg
output [7:0] result;


assign nCee = run_prog ? nCe:ram_sel;
assign nLm = run_prog?nLm_int:nLm_ext;



pc PC_sap1( .Cp(Cp), .nCLK(!clk), .nCLR(!clr), .Ep(Ep), .WBUSlower(WBUS[3:0]));



inputMAR MAR_sap1(.nLm(nLm), .clk(clk), .in(WBUS[3:0]), .ram_addr(ram_addr), .ram_nrd(ram_nrd), .ram_nwr(ram_nwr), .run_prog(run_prog));



ram ram_sap1(.clk(clk), .nrd(ram_nrd), .nwr(ram_nwr), .nce(nCee), .addr(ram_addr), .data(WBUS));



IR IR_sap1(.clk(clk), .nli(nLi), .nei(nEi), .clr(clr), .wbus(WBUS), .wbuslower(WBUS[3:0]), .wbusup(opcode)); //



control_unit cntrl_unit_sap1(clk, clr, cntrl_bus, clk_hlt, opcode);



accumulator ACC_sap1(.clk(clk), .nLa(nLa), .Ea(Ea), .accout(WBUS), .accin(WBUS), .accreg(ACC_to_ALU));



addsub addsub_sap1(.Su(Su), .Eu(Eu), .ina(ACC_to_ALU), .inb(regB_to_ALU), .result(WBUS));



regB regB_sap1(.nLb(nLb), .clk(clk), .in(WBUS), .out(regB_to_ALU));



outreg outreg_sap1(.nLo(nLo), .clk(clk), .in(WBUS), .out(result));

endmodule