
module InstructionFetch (clk,IpInstruction,IpPcB,OpInstruction,OpPc1,BranchOrNormalSelector,EnableIpToIrregister,hold);
input clk;
//Input
input [31:0] IpInstruction;
input [31:0] IpPcB;
//Extra (this part to make it able to take instructions from user (input))
reg [31:0] Ipc=0;		
input EnableIpToIrregister;			
//Intermediate
reg [31:0] IrMemory [0:1024];
reg [31:0] OpPc=0;
//reg [31:0] ImmediateValue;
//Output
output reg [31:0] OpInstruction;
output reg [31:0] OpPc1;
//ControlInput
input BranchOrNormalSelector;
input hold;
always @(posedge clk )
begin
OpPc1=OpPc+1;
if (EnableIpToIrregister==1)
begin
IrMemory[Ipc]=IpInstruction;
Ipc=Ipc+1;
end
if (IrMemory[OpPc]!=0)
begin
OpInstruction=IrMemory[OpPc];
end

#45
if (hold)
begin
OpPc=OpPc;
end
else
begin
if (BranchOrNormalSelector)
OpPc=IpPcB;
else 
OpPc=OpPc1;
end


end
initial 
begin

//IrMemory [0] = 32'b 100011_00000_00101_0000000000100100 ;//lw done
IrMemory [0] = 32'b 000000_00101_00111_00111_00000_100010 ;//sub done
IrMemory [1] = 32'b 000000_00110_00110_00110_00000_100000 ;///add done
/*IrMemory [3] = 32'b 000000_00110_00110_00110_00000_100000 ;///add done
IrMemory [4] = 32'b 000000_00001_00001_00001_00000_100000 ;
IrMemory [5] = 32'b 000000_00000_00110_00110_00000_100000 ;
IrMemory [4] = 32'b 000000_00001_00010_00000_00000_100101 ;//or done
IrMemory [5] = 32'b 000000_00111_00111_00100_00000_100000 ;///add done
IrMemory [6] = 32'b 101011_00001_00010_00000_00000_100110 ;//xor done 
IrMemory [7] = 32'b 001100_00001_00101_0100100000000100 ;//andi done
IrMemory [8] = 32'b 001110_00001_00010_0100100000000010 ;//xori done
IrMemory [9] = 32'b 001101_00001_01000_0000000000010100 ;//ori done
IrMemory [10] = 32'b 101011_00000_00111_0000000000100100 ;//sw done
IrMemory [11] = 32'b 000000_00111_00111_00100_00000_100000 ;///add done
IrMemory [12] = 32'b 100011_00000_00101_0000000000100100 ;//lw done
IrMemory [13] = 32'b 000000_00101_00101_00101_00000_100000 ;///add done*/

end
endmodule

module IF_ID_reg (clk,ipInstruction,ipPc1,opPc1,opInstruction,Repeat);
input clk;
//input
input [31:0] ipInstruction;
input [31:0] ipPc1;
input Repeat;
//output
output reg [31:0] opPc1;
output reg [31:0] opInstruction;
reg [63:0] If_Id_reg;
always @ (posedge clk)
begin
#60
if (Repeat)
begin

end
else
begin
If_Id_reg[63:32] = ipPc1;
If_Id_reg[31:0]  = ipInstruction;
opInstruction =  If_Id_reg[31:0];
opPc1 = If_Id_reg[63:32];
end
end
endmodule



module InstructionDecode (clk,Instruction,WriteData,RegWrite,WriteRegister,ReadData1,ReadData2,ImmediateValue  , Branch , MemRead , MemtoReg , MemWrite,ALUsrc,regWrite,RegDst,OpCode,MemReadH,Rd ,hold,Repeat);
input clk;
//inputs
input signed [31:0] Instruction;
input signed [31:0] WriteData;
input RegWrite;
input [4:0] WriteRegister;
//IntermediateRegisters
reg [4:0] ReadRegister1;
reg [4:0] ReadRegister2;
reg signed [31:0] RegisterFile [0:31];
reg indicator=0;
//outputs
output reg signed [31:0] ReadData1;
output reg signed [31:0] ReadData2;
output reg signed [31:0] ImmediateValue;
//ControlOutputs
output reg Branch ; 
output reg MemRead ; 
output reg MemtoReg ;
output reg MemWrite ;
output reg ALUsrc ;
output reg regWrite ;
output reg RegDst ;
output reg [5:0] OpCode;
//hazard stuff
input MemReadH ; //a control signal from the previous instruction, saved at Dec,Exe Reg
input [4:0] Rd ; //the destination reg of the prev instruction, saved at Dec,Exe Reg
//input [4:0] Rs ; //argument reg of the current instruction, saved at Fet,Dec Reg
//input [4:0] Rt ; //argument reg of the current instruction, saved at Fet,Dec Reg
output reg hold ; //to hold the PC at its current value without increment 
output reg Repeat ; //to process the same instruction again after stalling, by repeating the instruction saved in the Fet,Dec Reg
reg nop ; //to make the current operation act like "no operation" by making all the control signals = 0

always @ (posedge clk)
begin

if ( MemReadH == 1 && ((Rd == Instruction[20:16]) || (Rd == Instruction[25:21])) )
begin 
hold <= 1;
Repeat <= 1;
nop <= 1;

end

else
begin
hold <= 0;
Repeat <= 0;
nop <= 0;
end

end

always@(posedge clk)
begin

	ReadRegister1=Instruction[25:21];
        ReadRegister2=Instruction[20:16];
	ReadData2=RegisterFile[ReadRegister2];
	ReadData1=RegisterFile[ReadRegister1];
	ImmediateValue=Instruction[15:0];
	OpCode=Instruction[31:26];
if ( nop == 1 )
begin
 	      RegDst <= 1'b0 ;  // R-Format
	      Branch <= 1'b0 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b0 ;
	      regWrite <= 1'b0 ;
	      ALUsrc <= 1'b0 ;
	     indicator<=0;

end
else
begin
	if( OpCode ==6'b000000) 
	begin
 	      RegDst <= 1'b1 ;  // R-Format
	      Branch <= 1'b0 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b0 ;
	      regWrite <= 1'b1 ;
	      ALUsrc <= 1'b0 ;
	     indicator<=1;
	end

else if( OpCode == 6'b101011)  
	begin
	      RegDst <= 1'b0 ;  // SW
	      Branch <= 1'b0 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b1 ;
	      regWrite <= 1'b0 ;
	      ALUsrc <= 1'b1;
	      indicator <=1;
	end

else if( OpCode ==6'b100011) 
	begin
	      RegDst <= 1'b0 ;  // LW
	      Branch <= 1'b0 ;
	      MemRead <= 1'b1 ;
	      MemtoReg <= 1'b1 ;
	      MemWrite <= 1'b0 ;
	      regWrite <= 1'b1 ;
	      ALUsrc <= 1'b1;
	      indicator <=1;
	end

else if( OpCode ==6'b000100) 
	begin
	      RegDst <= 1'b0 ;  // beq
	      Branch <= 1'b1 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b0 ;
	      regWrite <= 1'b0 ;
	      ALUsrc <= 1'b0 ;
	      indicator<=1;
	end
else
	begin
	      RegDst <= 1'b0 ;  //immediate instructions
	      Branch <= 1'b0 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b0 ;
	      regWrite <= 1'b1 ;
	      ALUsrc <= 1'b1 ;
	      indicator<=1;
	end
end
	
end
always@(posedge clk)
begin

#10
	if(RegWrite == 1) 
	RegisterFile[WriteRegister]=WriteData;
	/*else
	RegisterFile[WriteRegister]=RegisterFile[WriteRegister];*/
	

	
	indicator=0;
end

always @ (posedge clk)
begin

if ( MemReadH == 1 && ((Rd == Instruction[20:16]) || (Rd == Instruction[25:21])) )
begin 
hold <= 1;
Repeat <= 1;
nop <= 1;
end

else
begin
hold <= 0;
Repeat <= 0;
nop <= 0;
end

end



initial 
begin

RegisterFile[0]=0;
RegisterFile[1]=25;
RegisterFile[2]=22;
RegisterFile[3]=2;
RegisterFile[4]=3;
RegisterFile[5]=5;
RegisterFile[6]=6;
RegisterFile[7]=28;
RegisterFile[8]=25;
RegisterFile[9]=43;
RegisterFile[10]=35;
RegisterFile[11]=52;
RegisterFile[12]=43;
RegisterFile[13]=75;
RegisterFile[14]=82;
RegisterFile[15]=245;
RegisterFile[16]=334;
RegisterFile[17]=512;
RegisterFile[18]=2343;
RegisterFile[19]=2395;
RegisterFile[20]=222;
RegisterFile[21]=213;
RegisterFile[22]=378;
RegisterFile[23]=567;
RegisterFile[24]=3456;
RegisterFile[25]=505;
RegisterFile[26]=2993;
RegisterFile[27]=2765;
RegisterFile[28]=2542;
RegisterFile[29]=278;
RegisterFile[30]=3986;
RegisterFile[31]=5467;

end
endmodule

module ID_EX(clk,in_instruction,opPc2,in_read_data1,out_read_data1,in_read_data2,out_read_data2,in_immediate_value,OutImmediateValue,InWriteRegister1,OutWriteRegister1,InWriteRegister2,OutWriteRegister2
,in_ALUsrc,in_ALUop,in_RegDst,in_MemRead,in_MemWrite,in_Branch,in_MemtoReg,in_RegWrite,out_ALUsrc,out_ALUop,out_RegDst,out_MemRead,out_MemWrite,out_Branch,out_MemtoReg,out_RegWrite);

//INPUTS
input clk; //done
input [31:0] in_instruction; //done
input signed [31:0] in_read_data1,in_read_data2,in_immediate_value; //done
input [4:0] InWriteRegister1;//done
input [4:0] InWriteRegister2;//done
input in_ALUsrc,in_RegDst,in_MemRead,in_MemWrite,in_Branch,in_MemtoReg,in_RegWrite;//done
input [5:0] in_ALUop;//done
//PIPELINE REGISTER
reg [255:0] ID_EX_register;

//OUTPUTS
output reg [31:0] opPc2;//done
output reg signed [31:0] out_read_data1,out_read_data2,OutImmediateValue;//done
output reg [4:0] OutWriteRegister1;//done
output reg [4:0] OutWriteRegister2;//done
output reg out_ALUsrc,out_RegDst,out_MemRead,out_MemWrite,out_Branch,out_MemtoReg,out_RegWrite;//done
output reg [5:0] out_ALUop;//done
always @(posedge clk)
begin
#60
	ID_EX_register[31:0]    = in_instruction ;
	ID_EX_register[63:32]   = in_read_data1 ;
	ID_EX_register[95:64]   = in_read_data2 ;
	ID_EX_register[127:96]  = in_immediate_value ;
	ID_EX_register[132:128] = InWriteRegister1;
	ID_EX_register[136:133] = InWriteRegister2;
	ID_EX_register[137]     =     in_ALUsrc;//ex
	ID_EX_register[143:138] =     in_ALUop;//ex
	ID_EX_register[144]     =     in_RegDst;//ex
	ID_EX_register[145]     =     in_MemRead;//Dmem
	ID_EX_register[146]     =     in_MemWrite;//Dmem
	ID_EX_register[147]     =     in_Branch;//Dmem
	ID_EX_register[148]     =     in_MemtoReg;//wb
	ID_EX_register[149]     =     in_RegWrite;//wb
	opPc2		         = ID_EX_register[31:0] ;    
	out_read_data1          = ID_EX_register[63:32] ;   
	out_read_data2          = ID_EX_register[95:64] ; 
	OutImmediateValue     = ID_EX_register[127:96] ; 
	OutWriteRegister1       = ID_EX_register[132:128] ; 
	OutWriteRegister2       = ID_EX_register[136:133] ; 
	out_ALUsrc 		= ID_EX_register[137]  ;
	out_ALUop		= ID_EX_register[143:138] ;
	out_RegDst 		= ID_EX_register[144]  ;
	out_MemRead 		=ID_EX_register[145] ;
	out_MemWrite 		 =ID_EX_register[146] ;
	out_Branch 		= ID_EX_register[147] ; 
	out_MemtoReg 		= ID_EX_register[148] ;
	out_RegWrite 		= ID_EX_register[149] ;

end
endmodule


module ALU_processor(clk,A,B1,ImmediateValue,OpPc1,op,func,WriteRegister1,WriteRegister2,Result,IpPcB,Zero_flag,WriteRegister,ALUsrc,RegDst);

input clk;
//Input
input signed [31:0] A;
input signed [31:0] B1;
input signed [31:0] ImmediateValue;
input [31:0] OpPc1;
input [5:0] op;
input [5:0] func;
input [4:0] WriteRegister1;
input [4:0] WriteRegister2;
//Output
output reg signed [31:0] Result;
output reg [31:0] IpPcB;	
output reg Zero_flag;
output reg [4:0]WriteRegister;
//output reg BranchOrNormalSelector;
//intermediate
reg signed [31:0] B;
//control
//input Branch;
input ALUsrc;
input RegDst;
always @(posedge clk)
begin 
	
	IpPcB=ImmediateValue+OpPc1;
	if (ALUsrc)  //watchout
	B=ImmediateValue;
	else 
	B=B1;
	if(op != 6'b000100)
	begin
		 Result   <= 	(op == 6'b000000 && func == 6'b100000)? (A+B): //ADD
				(op == 6'b000000 && func == 6'b100010)? (A-B): //SUB 
				(op == 6'b000000 && func == 6'b100100)? (A&B): // AND 
				(op == 6'b000000 && func == 6'b100101)? (A|B): // OR	
				(op == 6'b000000 && func == 6'b100111)? !(A|B): // NOR
				(op == 6'b000000 && func == 6'b100110)? (A^B): // XOR
				(op == 6'b000000 && func == 6'b101010)? ( (A < B )? 1'b1 : 1'b0 ): //SLT
				(op == 6'b001000)? (A+B): //ADDI
				(op == 6'b001100)? (A&B): //ANDI
				(op == 6'b001110)? (A^B): //XORI
				(op == 6'b001101)? (A|B): //ORI
				(op == 6'b100011)? (A+B): //LW
				(op == 6'b101011)? (A+B): //SW
				(op == 6'b001010)? ( (A < B )? 1'b1 : 1'b0 ): 1'bx ; //SLTI
	end
					
	else if	(op == 6'b000100) begin Zero_flag <= (A==B)? 1'b1 : 1'b0 ; end //BQE

	if (RegDst)
	WriteRegister=WriteRegister2;//Instruction[15:11];
	else
	WriteRegister=WriteRegister1;//Instruction[20:16];
	/*if (Branch&&Zero_flag)
	BranchOrNormalSelector=1;
	else
	BranchOrNormalSelector=0;*/
end
endmodule



module EX_MEM_reg(clk,in_result,out_result,in_read_data2,out_read_data2,in_zero_flag,out_zero_flag,in_branch,out_branch,in_register2,out_register2,
                    in_MemRead,in_MemWrite,in_Branch,in_MemtoReg,in_RegWrite,out_MemRead,out_MemWrite,out_Branch,out_MemtoReg,out_RegWrite);

//INPUTS
input clk;
input [31:0] in_branch;
input signed [31:0] in_result,in_read_data2;
input in_zero_flag;
input [4:0] in_register2;
//PIPELINE REG
reg [255:0] EX_MEM_Register;
//OUTPUTS
output reg signed [31:0] out_result,out_read_data2;
output reg [31:0] out_branch;
output reg [4:0] out_register2;
output reg out_zero_flag;
//control inputs
input in_MemRead,in_MemWrite,in_Branch,in_MemtoReg,in_RegWrite;
//control outputs
output reg out_MemRead,out_MemWrite,out_Branch,out_MemtoReg,out_RegWrite;

always @(posedge clk)
begin

	#60
	EX_MEM_Register[63:32]   = in_result ;
	EX_MEM_Register[95:64]   = in_branch ;
	EX_MEM_Register[127:96]  = in_read_data2;
	EX_MEM_Register[128]     = in_zero_flag ;
	EX_MEM_Register[133:129] = in_register2 ;
	EX_MEM_Register[145]     =     in_MemRead;//Dmem
	EX_MEM_Register[146]     =     in_MemWrite;//Dmem
	EX_MEM_Register[147]     =     in_Branch;//Dmem
	EX_MEM_Register[148]     =     in_MemtoReg;//wb
	EX_MEM_Register[149]     =     in_RegWrite;//wb
	out_result     = EX_MEM_Register[63:32];
	out_branch     = EX_MEM_Register[95:64];
	out_read_data2 = EX_MEM_Register[127:96];
	out_zero_flag  = EX_MEM_Register[128];
	out_register2  = EX_MEM_Register[133:129];
	out_MemRead  =EX_MEM_Register[145] ;
	out_MemWrite 	= EX_MEM_Register[146] ;
	out_Branch 	= EX_MEM_Register[147] ; 
	out_MemtoReg 	= EX_MEM_Register[148] ;
	out_RegWrite 	= EX_MEM_Register[149] ;
	
	
end
 
endmodule


module DataMemory (clk,memwrite_enable,memread_enable,Zero,Branch,IPaddress,write_data,read_data ,BranchOrNormalSelector,OPaddress);
input clk;
//input
input [31:0] IPaddress;
input signed [31:0] write_data;
//Control
input memread_enable;
input memwrite_enable;
input Zero;
input Branch;
//Output
output reg signed  [31:0] read_data;
output  reg BranchOrNormalSelector;
output reg [31:0] OPaddress;
//IntermediateRegister
reg signed[31:0] Data_memory [0:55000] ;
always @ ( posedge clk )

	begin
		OPaddress = IPaddress;
		if (Zero&&Branch)
		BranchOrNormalSelector=1;
		else
		BranchOrNormalSelector=0;


		if ( memwrite_enable == 1 ) 
				Data_memory [IPaddress] <= write_data ;
		if ( memread_enable == 1  )
				read_data <= Data_memory[IPaddress] ;
			
	end
initial
begin
Data_memory[0]=5321;
Data_memory[1]=3123;
Data_memory[2]=2315;
Data_memory[3]=3133;
Data_memory[4]=54657;
Data_memory[5]=3897;
Data_memory[6]=56757;
Data_memory[7]=10973;
Data_memory[8]=45675;
Data_memory[9]=5783;
Data_memory[18471]=22;


end
		
endmodule

module Mem_WB_reg (clk,inMemoryToRegister,outMemoryToRegister,inReadData,outReadData,inAluResult,outAluResult,
		   inWriteRegister,outWriteRegister,IPRegWrite,OPRegWrite) ;

input clk ;
//input
input inMemoryToRegister ;
input signed [31:0] inReadData ;
input signed [31:0] inAluResult ;
input [4:0] inWriteRegister ;
input IPRegWrite;
//output
output reg outMemoryToRegister ;
output reg signed [31:0] outReadData ;
output reg signed [31:0] outAluResult ;
output reg [4:0] outWriteRegister ;
output reg OPRegWrite;
//intermediateRegister
reg [255:0] Mem_Wb_Reg;
always @ (posedge clk)
begin
#60
Mem_Wb_Reg[0]=inMemoryToRegister;
Mem_Wb_Reg[32:1]=inReadData;
Mem_Wb_Reg[64:33]=inAluResult;
Mem_Wb_Reg[68:65]=inWriteRegister;
Mem_Wb_Reg[69]=IPRegWrite;

outMemoryToRegister = Mem_Wb_Reg[0];
outReadData =  Mem_Wb_Reg[32:1];
outAluResult = Mem_Wb_Reg[64:33];
outWriteRegister =Mem_Wb_Reg[68:65];
OPRegWrite=Mem_Wb_Reg[69];
end
endmodule 



module WriteBackStage (clk,ReadData,AluResult,MemoryToRegister,WriteData);
input clk;
//Input
input signed [31:0] ReadData;
input signed [31:0] AluResult;
//ControlInput
input MemoryToRegister;
//Output
output reg signed [31:0] WriteData;

always @(posedge clk)
begin

if (MemoryToRegister==0)

WriteData=AluResult;

if (MemoryToRegister==1)

WriteData=ReadData;

end
endmodule 


module Processor_test;
//outputs from InstructionFetch
wire [31:0]OpInstruction;
wire [31:0] OpPc1;
//outputs from IF_ID_REG
wire [31:0] opPc1;
wire [31:0]opInstruction;
//outputs from InstructionDecode
wire signed [31:0]ReadData1;
wire signed [31:0]ReadData2;
wire signed [31:0] ImmediateValue;
wire Branch;
wire Memread;
wire MemtoReg;
wire Memwrite;
wire ALUsrc;
wire regWrite;
wire RegDst;
wire [5:0] OpCode;
wire hold;
wire Repeat;
//output from ID_EX_REG
wire signed [31:0]out_read_data1;
wire signed [31:0]out_read_data2;
wire signed [31:0]OutImmediateValue;
wire [31:0] opPc2;
wire [4:0] OutWriteRegister1;
wire [4:0] OutWriteRegister2;
wire out_ALUsrc,out_RegDst,out_MemRead,out_MemWrite,out_Branch,out_MemtoReg,out_RegWrite;
wire [5:0] out_ALUop;
//outputs from Alu
wire signed [31:0] AluResult ;
wire [31:0] IpPcB;
wire  Zero_flag;
wire [4:0]WriteRegister;
//outputs from EX_DM
wire signed [31:0]out_result;
wire signed [31:0]Out_read_data2;
wire out_zero_flag;
wire [31:0]out_branchPC;
wire [4:0]out_WriteRegister;
wire out_MemRead2,out_MemWrite2,out_Branch2,out_MemtoReg2,out_RegWrite2;
//outputs from DataMemory
wire signed [31:0] read_data;
wire BranchOrNormalSelector;
wire signed [31:0] OPaddress;
//outputs from Mem_WB_reg
wire signed [31:0] outReadData;
wire signed [31:0] outAluResult;
wire [4:0] outWriteRegister2;
wire OPRegWrite;
wire outMemoryToRegister;
//outputs from WriteBack
wire signed [31:0] WriteData;
//inputs for whole processor
reg clk=0;
reg [5:0] PositiveEdgeOfClkCounter=0;
reg signed[31:0] IpInstruction;
reg EnableIpToIrregister;

InstructionFetch  F1 		(clk,IpInstruction,out_branchPC,OpInstruction,OpPc1,BranchOrNormalSelector,EnableIpToIrregister,hold);
IF_ID_reg 	  F_D_reg1	(clk,OpInstruction,OpPc1,opPc1,opInstruction,Repeat);
InstructionDecode D1		(clk,opInstruction,WriteData,OPRegWrite,outWriteRegister2,ReadData1,ReadData2,ImmediateValue  , Branch , MemRead , MemtoReg , MemWrite,ALUsrc,regWrite,RegDst,OpCode,out_MemRead,OutWriteRegister1 ,hold,Repeat);
ID_EX             D_EX1		(clk,opPc1,opPc2,ReadData1,out_read_data1,ReadData2,out_read_data2,ImmediateValue,OutImmediateValue,opInstruction[20:16],OutWriteRegister1,opInstruction[15:11],OutWriteRegister2
				,ALUsrc,OpCode,RegDst,MemRead,MemWrite,Branch,MemtoReg,regWrite,out_ALUsrc,out_ALUop,out_RegDst,out_MemRead,out_MemWrite,out_Branch,out_MemtoReg,out_RegWrite);
ALU_processor	  ALU1		(clk,out_read_data1,out_read_data2,OutImmediateValue,opPc2,out_ALUop,OutImmediateValue[5:0],OutWriteRegister1,OutWriteRegister2,AluResult,IpPcB,Zero_flag,WriteRegister,out_ALUsrc,out_RegDst);
EX_MEM_reg	  EX_DM_reg1	(clk,AluResult,out_result,out_read_data2,Out_read_data2,Zero_flag,out_zero_flag,IpPcB,out_branchPC,WriteRegister,out_WriteRegister,
				out_MemRead,out_MemWrite,out_Branch,out_MemtoReg,out_RegWrite,out_MemRead2,out_MemWrite2,out_Branch2,out_MemtoReg2,out_RegWrite2);
DataMemory 	  DM1		(clk,out_MemWrite2,out_MemRead2,out_zero_flag,out_Branch2,out_result,Out_read_data2,read_data ,BranchOrNormalSelector,OPaddress);
Mem_WB_reg 	  M_W_r1	(clk,out_MemtoReg2,outMemoryToRegister,read_data,outReadData,OPaddress,outAluResult,out_WriteRegister,outWriteRegister2,out_RegWrite2,OPRegWrite) ;
WriteBackStage 	  WBS1		(clk,outReadData,outAluResult,outMemoryToRegister,WriteData);
always
#30
clk=~clk;
always@(posedge clk)
PositiveEdgeOfClkCounter=PositiveEdgeOfClkCounter+1;
initial
begin
$monitor("PositiveEdgeOfClkCounter= %d  AluResult=%d  WriteData=%d  OpInstruction= %b  ReadData1=%d ReadData2=%d regwrite=%d ",PositiveEdgeOfClkCounter,AluResult,WriteData,opInstruction,ReadData1,ReadData2,out_RegWrite2);
end
endmodule



