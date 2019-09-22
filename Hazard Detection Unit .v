module HazardDetector ( clk,MemRead,Rd,Rs,Rt ,hold,Repeat,nop );

input clk ;
input MemRead ; //a control signal from the previous instruction, saved at Dec,Exe Reg
input [4:0] Rd ; //the destination reg of the prev instruction, saved at Dec,Exe Reg
input [4:0] Rs ; //argument reg of the current instruction, saved at Fet,Dec Reg
input [4:0] Rt ; //argument reg of the current instruction, saved at Fet,Dec Reg
output reg hold ; //to hold the PC at its current value without increment 
output reg Repeat ; //to process the same instruction again after stalling, by repeating the instruction saved in the Fet,Dec Reg
output reg nop ; //to make the current operation act like "no operation" by making all the control signals = 0

always @ (posedge clk)
begin

if ( MemRead == 1 && ((Rd == Rs) || (Rd == Rt)) )
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

endmodule

module hazardDetec_test ;

reg MemRead ; reg[4:0] Rd ; reg [4:0] Rs ; reg [4:0] Rt ;
wire hold ; wire Repeat ; wire nop ;
reg clk = 1 ;

HazardDetector my_hazardDetector ( clk,MemRead,Rd,Rs,Rt ,hold,Repeat,nop );

always 
begin
#1 clk = ~clk ;
end

initial
begin 

#2
MemRead = 1 ;
Rd = 23 ;
Rs = 23 ;
Rt = 15 ;
$display ( $time,,,"hold = %b , Repeat = %b , nop = %b",hold,Repeat,nop) ;
#2
MemRead = 0 ;
Rd = 23 ;
Rs = 23 ;
Rt = 15 ;
$display ( $time,,,"hold = %b , Repeat = %b , nop = %b",hold,Repeat,nop) ;
#2
MemRead = 1 ;
Rd = 23 ;
Rs = 15 ;
Rt = 23 ;
$display ( $time,,,"hold = %b , Repeat = %b , nop = %b",hold,Repeat,nop) ;
#2
MemRead = 1 ;
Rd = 15 ;
Rs = 23 ;
Rt = 23 ;
$display ( $time,,,"hold = %b , Repeat = %b , nop = %b",hold,Repeat,nop) ;
#2
MemRead = 1 ;
Rd = 23 ;
Rs = 23 ;
Rt = 23 ;
$display ( $time,,,"hold = %b , Repeat = %b , nop = %b",hold,Repeat,nop) ;

end

endmodule
