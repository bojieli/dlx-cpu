/* Instruction Fetch */
// ==============================================================================================

/*
 * Branch Predictor -- abbreviated as BP
 * Program Counter -- abbreviated as PC
 */

/*
 * Data flow for PC:
 * 
 * Variables Table:
 * iq_size Instruction                  Queue Size, which is fixed.
 * commit_instruction_num               The instructions that are committed one time in IQ.
 * header                                               the header of IQ.
 * tail                                                 the tail of IQ.
 *
 */
Input: head, tail, commit_instruction_num
Output: PC value for IMEM
always @ (posedge clk)
begin 
        // If there is still room in IQ, fetch another instructions, otherwise, do
        // nothing.
        if ((iq_size + commit_instruction_num - (header - tail)) >= 1)
        {
                // If current PC value is in the Branch Target Buffer(hereinafter
                // referredas BTB, then use the target address in the BTB to update PC.
                // Otherwise, PC <- NPC.
                if (BP[PC].valid == 1)
                        begin
                        PC = BP[PC].targetAddress;
                        end
                else
                        begin
                                PC = PC + 4;
                        end
        }
end

/*
 * Instruction Memory -- abbreviated as IMEM
 * Data flow for IMEM.
 * This part is combinational logic.
 */
Input: PC
Output: IMEM[PC]

/*
 * Decode Module
 * This module decode the Instruction fetched from IMEM.
 * This module is combinational.
 */
Input: IMEM[PC] (hereinafter referred as IR)
Output: DI(abbreviation for DI(Decoded Instruction)
always @(*)
begin
        DI.op = IR.op;
        DI.use_rs1 = 1;
        DI.rs1_addr = IR.rs1;
        if (IR.op belong to alu)
        {
                DI.type = alu;
                DI.use_rd = 1;
                DI.rd_addr = IR.rd
                if (IR.op is R-type)
                {
                        DI.use_rs2 = 1;
                        DI.rs2_addr = IR.rs2;
                        DI.use_imm = 0;
                }
                else
                {
                        DI.use_rs2 = 0;
                        DI.use_imm = 1;
                }
        }
        if (IR.op belong to mem)
        {
                DI.type = mem;
                DI.use_rd = 1;
                DI.rd_addr = IR.rd
                DI.use_rs2 = 0;
                DI.use_imm = 1;
        }
        if (IR.op is jmp)
        {
                DI.type = jmp;
                DI.use_rs1 = 0;
                DI.use_rs2 = 0;
                DI.use_rd = 0;
        }
        /* TODO: decoder for jump instrution*/
        if (IR.op is jr)
        {
                /* code */
        }
end

// ==============================================================================================
/* Issue */
/*
 * Instruction Queue -- abbreviated as IQ
 * Data flow for IQ
 */


/* Instrunction Queue
   QUEUE_SIZE is the size of queue
   QUEUE_ADDR_SIZE is log2(QUEUE_SIZE)
   queue_free_space is the free space of the queue
 */
module iq();

   parameter QUEUE_ADDR_SIZE = 3;
   parameter QUEUE_SIZE = 8;

   assign queue_free_space = QUEUE_SIZE - 1 + commit - (tail - head);
   assign use_instrunction_1 = !flush && (queue_free_space >= 1);
   assign use_instrunction_2 = !flush && (queue_free_space >= 2);

   always@(posedge clk)
     begin
        for( i = 0; i < commit_instrunction_num; i = i + 1 )
          queue[head+i].valid <= 0;

        head <= head + commit_instrunction_num;

        if( flush )
          tail <= tail_flush_new;
        else
          tail <= tail + use_instrunction_1 + use_instrunction_2;

        if( use_instrunction_1 )
          queue[tail  ] <= instrunction_1;
        if( use_instrunction_2 )
          queue[tail+1] <= instrunction_2;

     end

endmodule // iq



/* make schedule for memory type instrunction
 * ready_mem[x] == 1 means the instrunction IQ[x] is ready and instrunction is memory type
 * 
 * use_mem == 1 means one instrunction is scheduled
 * instrunction_index is the index of the scheduled instrunction
 */
module schedule_mem();
   input  [QUEUE_SIZE-1:0]ready_mem;

   output use_mem;
   output [QUEUE_ADDR_SIZE-1:0] instrunction_index;
   
   always@(head or ready_mem)
     begin
        if( ready_mem == 0 )
          use_mem <= 0;
        else
          begin
             use_mem <= 1;
             instrunction_index <= x where (ready_mem[x] == 1) and ((x - head) is minimal);
          end
     end
endmodule // schedule_mem

/* make schedule for alu or jump type instrunction
 * ready_aj[x] == 1 means the instrunction IQ[x] is ready and instrunction is alu or jump type
 * 
 * use_alu_1 == 1 means at least one instrunction is scheduled
 * use_alu_2 == 1 means two instrunction is scheduled
 * alu_1_instrunction_index is the index of the first scheduled instrunction
 * alu_2_instrunction_index is the index of the second scheduled instrunction
 */
module schedule_aj();
   input  [QUEUE_SIZE-1:0]ready_aj;

   output use_alu_1;
   output use_alu_2;
   output [QUEUE_ADDR_SIZE-1:0] alu_1_instrunction_index;
   output [QUEUE_ADDR_SIZE-1:0] alu_2_instrunction_index;
   
   always@(head or ready_mem)
     begin
        if( ready_aj == 0 )
          use_alu_1 <= 0
        else
          begin
             use_alu_1 <= 1;
             alu_1_instrunction_index <= x where (ready_aj[x] == 1) and ((x - head) is minimal);
             if( ready_aj & ~(1<<alu_1_instrunction_index) == 0 )
               use_alu_2 <= 0;
             else
               begin
                  use_alu_2 <= 1;
                  alu_2_instrunction_index <= x where (ready_aj[x] == 1) and ((x - head) is secondary minimal);
               end
          end
     end
endmodule // schedule_aj

/* generate the command of memory instrunction
 *
 */
module make_memory_command();
   input [QUEUE_ADDR_SIZE-1:0] instrunction_index;

   assign inst_id = instrunction_index;
   assign inst    = IQ[inst_id];
   assign load_or_store = (inst.type == "load");
   assign base_addr_readreq.use_reg   = 1;
   assign base_addr_readreq.reg_index = inst.rs1_addr;
   assign offset  = inst.imm;
endmodule // make_memory_command

/* generate the command of alu or jump instrunction
 *
 */
module make_alu_jump_command();
   input [QUEUE_ADDR_SIZE-1:0] instrunction_index;

   assign inst_id = instrunction_index;
   assign inst    = IQ[inst_id];
   assign operation = inst.op;
   assign is_jump   = (inst.type == "jump");
   assign is_branch = (inst.type == "branch");

   assign arg1_readreq.use_reg = use_rs1;
   assign arg2_readreq.use_reg = use_rs2;

   assign arg1_readreq.reg_index = rs1_addr;
   assign arg2_readreq.reg_index = rs2_addr;
   
   assign arg2_readreq.imm = inst.imm;

   assign next_pc = inst.next_pc;
endmodule // make_alu_jump_command

/*
 * This part is the submodule of IQ to compute the dependence between the
 * instruction that just fetched and all the instructions already stored in the
 * IQ.
 * Scan all Instructions in IQ and compute the depedence, which is done
 * parallelly.
 * The depedence information is stored in a two dimension array(literally). The
 * Dep[i][j] = 1 means IQ[i] depends IQ[j].
 */
Input: DI(abbreviated as DI(Decoded Instruction) in the pseudo-code description)
Output:
always @ (posedge clk)
        begin
                if (DI.use_rs1 == 1)
                {
                        for (int i = head; i < tail; i++)
                                {
                                        if (IQ[i].rd_use == 1 && IQ[i].rd == DI.rs1_addr)
                                        {
                                                Dep[tail][i] = 1;
                                        }
                                }
                }
                if (DI.use_rs2 == 1)
                {
                        for (int i = head; i < tail; i++)
                                {
                                        if (IQ[i].rd_use == 1 && IQ[i].rd == DI.rs2_addr)
                                        {
                                                Dep[tail][i] = 1;
                                        }
                                }
                }
        end

// ==============================================================================================
/* Execution */
/*
 * This module does the actual execution. There are several submodules, which
 * are described in the following:
 * Every function unit has a control unit associated with it. For example, one
 * alu has a cmd_alu stored the control signal and other necessery information.
 */


/*
 * inst                 the actual instruction.
 * pc_predict   the predicted PC in the IF stage.
 */
/*
 * Operation of function unit.
 */
Input: signal from cmd_alu

Output: rd_value
always @(*)
        begin
                switch(based on inst.func and inst.op)
                        {
                                /* do actual operations,like add, sub etc. */
                                case add:
                                        {
                                                inst.rd_value = inst.rs1_value + inst.rs2_value;
                                        }
                                case: jmp:
                                          {
                                                /* TODO: the operation for branch instructions*/
                                          }
                                
                        }
        end


// ==============================================================================================
/*
 * Commit Stage of our CPU
 */
// Versioned Register File
// includes ROB (Reorder Buffer) and Registers.

module vreg(rst, clk, flu, iq_new_tail, iq_head, iq_tail, iq_finish, iq_commit, r1_alu, r2_alu, r_mem, w_alu, w_mem);

// ROB: each entry in instruction queue has a 32-bit buffer for saving temporary result between Finish and Commit.
// Registers: 32 general-purpose 32-bit registers whose values are saved when instruction commits.

// always:
//     {ROB, regfile} => r1_alu, r2_alu, r_mem

// on Finish:
//         {w_alu, w_mem} => ROB

// on Commit:
//         ROB => regfile

input rst, clk, flu;
input iq_new_tail[INST_QUEUE_SIZE]; // new tail in instruction queue when flush
input iq_head[INST_QUEUE_SIZE]; // head of instruction queue
input iq_tail[INST_QUEUE_SIZE]; // tail of instruction queue

input iq_finish[INST_QUEUE_SIZE]; // the instruction is finished this cycle (to be saved to ROB)
input iq_commit[INST_QUEUE_SIZE]; // the instruction should be commited this cycle (to be saved to register file)

#define WORD_SIZE 32
#define REG_ADDR_WIDTH 5
#define REG_NUM (1<<REG_ADDR_WIDTH)
#define ALU_NUM 2
#define INST_QUEUE_SIZE 8

struct read {
        output data[WORD_SIZE];
        input addr[REG_ADDR_WIDTH]; // register address
        input iq_pos[INST_QUEUE_SIZE]; // position in instruction queue
};
struct write {
        input data[WORD_SIZE];
        input addr[REG_ADDR_WIDTH]; // register address
        input iq_pos[INST_QUEUE_SIZE]; // position in instruction queue
};

struct read r1_alu[ALU_NUM], r2_alu[ALU_NUM]; // each ALU needs to read two data
struct read r_mem; // from register to memory
struct write w_alu[ALU_NUM]; // each ALU outputs one result
struct write w_mem; // from memory to register

reg rob[INST_QUEUE_SIZE][WORD_SIZE]; // result buffer
reg rob_valid[INST_QUEUE_SIZE]; // is rob valid? (between Finish and Commit)
reg rob_commit_addr[INST_QUEUE_SIZE][REG_ADDR_WIDTH]; // write address for each instruction in IQ when it commits

reg regfile[REG_NUM][WORD_SIZE]; // register file after commit

function _read_data(struct read r)
begin
        assign r.data = [
                (rob_valid[t] ? rob[t].data : regfile[r.addr])
                for t in range(0,INST_QUEUE_SIZE) 
                if r.iq_pos == t
        ].join(or);
end

foreach (i in range(0,ALU_NUM))
begin
        _read_data(r1_alu[i]);
        _read_data(r2_alu[i]);
end

_read_data(r_mem);

always@(posedge clk or negedge rst)
        if (flu)
                foreach (i in range(0,INST_QUEUE_SIZE))
                        if (iq_new_tail <= iq_tail ?
                                  (iq_new_tail <= i && i < iq_tail)
                                : (iq_new_tail <= i || i < iq_tail))
                        begin
                                rob_valid[i] <= 0;
                        end

        else foreach (i in range(0,INST_QUEUE_SIZE))
        begin
                if (iq_finish[i])
                begin
                        rob[i] <= [w_alu[j].data for j in range(0,ALU_NUM) if w_alu[j].iq_pos == i].join(or) 
                                or (w_mem.data if w_mem.iq_pos);
                        rob_valid[i] <= 1;
                        rob_commit_addr[i] <= w_alu[j].addr;
                end

                if (iq_commit[i])
                begin
                        rob_valid[i] <= 0;
                        regfile[rob_commit_addr[i]] <= rob[i];
                end
        end

endmodule
