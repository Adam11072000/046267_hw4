/* 046267 Computer Architecture - Winter 20/21 - HW #4 */

#include "core_api.h"
#include "sim_api.h"
#include <vector>

using std::vector;

class Thread{
    int halt_counter;
    vector<int> regs;
    bool finished;
    int curr_line;
public:
    Thread();
    ~Thread() = default;
    Thread(const Thread& src) = default;
    void set_halt_counter(int c);
    int get_halt_counter() const;
    bool is_finished() const;
    void set_finished();
    void set_reg(int r, int v); // 0 <= r <= 7
    int get_reg(int r);
    int get_curr_line() const;
    void inc_curr_line();
};

Thread::Thread(): halt_counter(0), regs(vector<int>(REGS_COUNT, 0)), finished(false), curr_line(0) {}

void Thread::set_halt_counter(int c) {
    halt_counter = c;
}

int Thread::get_halt_counter() const {
    return halt_counter;
}

bool Thread::is_finished() const {
    return finished;
}

void Thread::set_finished() {
    finished = true;
}

void Thread::set_reg(int r, int val) {
    regs[r] = val;
}

int Thread::get_reg(int r) {
    return regs[r];
}

int Thread::get_curr_line() const {
    return curr_line;
}

void Thread::inc_curr_line() {
    curr_line++;
}

class Core{
    vector<Thread> threads;
    int cs_overhead;
    int load_cycles;
    int store_cycles;
    int total_cycles;
    int total_insts;
    int current_thread;
    bool is_blocked_mt;
	int thread_num;

public:
    Core(int threads_num, int cs_oh, int lw_cycles, int sw_cycles, bool is_blocked);
    ~Core() = default;
    Core(const Core& src) = default;
	void run_sim();
	void next_cycle();
	void execute_cmd();
	int find_next_thread_to_run();
	double get_cpi() const;
    void fill_thread_context(tcontext* context, int tid);
};

Core::Core(int threads_num, int cs_oh, int lw_cycles, int sw_cycles, bool is_blocked): threads(vector<Thread>(threads_num)),
cs_overhead(cs_oh), load_cycles(lw_cycles), store_cycles(sw_cycles), total_cycles(0), total_insts(0), current_thread(0),
is_blocked_mt(is_blocked), thread_num(threads_num){}

double Core::get_cpi() const {
	return (double)total_cycles/ (double) total_insts;
}

void Core::fill_thread_context(tcontext *context, int tid) {
	for(int i = 0; i < REGS_COUNT; i++){
		context[tid].reg[i] = threads[tid].get_reg(i);
	}
}

void Core::run_sim() {
	while(true){
		while(!threads[current_thread].is_finished() && threads[current_thread].get_halt_counter() == 0){
			execute_cmd();
			next_cycle();
			if(threads[current_thread].get_halt_counter() != 0 || !is_blocked_mt){
				break;
			}
		}
		current_thread = find_next_thread_to_run();
		if(current_thread == -1){
			break;
		}
	}
}

void Core::execute_cmd() {
	if(threads[current_thread].is_finished()){
		return;
	}
	int res;
	Instruction curr_ins;
	SIM_MemInstRead(threads[current_thread].get_curr_line(), &curr_ins, current_thread);
	if (curr_ins.opcode == CMD_HALT) {
		if (!threads[current_thread].is_finished()) {
			threads[current_thread].set_finished();
		}
	} else if (curr_ins.opcode == CMD_NOP) {

	} else if (curr_ins.opcode == CMD_ADD) {
		res = threads[current_thread].get_reg(curr_ins.src1_index)
			  + threads[current_thread].get_reg(curr_ins.src2_index_imm);
		threads[current_thread].set_reg(curr_ins.dst_index, res);
	} else if (curr_ins.opcode == CMD_SUB) {
		res = threads[current_thread].get_reg(curr_ins.src1_index)
			  - threads[current_thread].get_reg(curr_ins.src2_index_imm);
		threads[current_thread].set_reg(curr_ins.dst_index, res);
	} else if (curr_ins.opcode == CMD_ADDI) {
		res = threads[current_thread].get_reg(curr_ins.src1_index) + curr_ins.src2_index_imm;
		threads[current_thread].set_reg(curr_ins.dst_index, res);
	} else if (curr_ins.opcode == CMD_SUBI) {
		res = threads[current_thread].get_reg(curr_ins.src1_index) - curr_ins.src2_index_imm;
		threads[current_thread].set_reg(curr_ins.dst_index, res);
	} else if (curr_ins.opcode == CMD_LOAD) {
		threads[current_thread].set_halt_counter(load_cycles + 1);
		uint32_t addr = threads[current_thread].get_reg(curr_ins.src1_index);
		if (curr_ins.isSrc2Imm) {
			addr += curr_ins.src2_index_imm;
		} else {
			addr += threads[current_thread].get_reg(curr_ins.src2_index_imm);
		}
		int tmp_val = 0;
		SIM_MemDataRead(addr, &tmp_val);
		threads[current_thread].set_reg(curr_ins.dst_index, tmp_val);
	} else {
		threads[current_thread].set_halt_counter(store_cycles + 1);
		uint32_t addr = threads[current_thread].get_reg(curr_ins.dst_index);
		if (curr_ins.isSrc2Imm) {
			addr += curr_ins.src2_index_imm;
		} else {
			addr += threads[current_thread].get_reg(curr_ins.src2_index_imm);
		}
		SIM_MemDataWrite(addr, threads[current_thread].get_reg(curr_ins.src1_index));
	}
	threads[current_thread].inc_curr_line();
	total_insts++;
}

void Core::next_cycle() {
	total_cycles++;
	for(int i = 0; i < thread_num; i++){
		if(!threads[i].is_finished() && threads[i].get_halt_counter() != 0){
			threads[i].set_halt_counter(threads[i].get_halt_counter() - 1);
		}
	}
}

int Core::find_next_thread_to_run() {
	bool is_done = true;
	for(int i = 0; i < thread_num; i++){
		if(!threads[i].is_finished()){
			is_done = false;
		}
	}
	if(is_done){
		return -1;
	}
	int idx, to_return = -1;
	while(to_return == -1) {
		for (int i = 0; i < thread_num; i++) {
			if(is_blocked_mt){
				idx = (i + current_thread) % thread_num;
			}else{
				idx = (i + current_thread + 1) % thread_num;
			}
			if (!threads[idx].is_finished() && threads[idx].get_halt_counter() == 0) {
				to_return = idx;
				break;
			}
		}
		if (idx != current_thread && is_blocked_mt && to_return != -1) {
			for (int i = 0; i < cs_overhead; i++) {
				next_cycle();
			}
		}
		if(to_return == -1) {
			next_cycle();
		}
	}
	return to_return;
}

Core* BlockedMTCore = nullptr;
Core* FineGrainedCore = nullptr;

void CORE_BlockedMT() {
	BlockedMTCore = new Core(SIM_GetThreadsNum(), SIM_GetSwitchCycles(),
							 SIM_GetLoadLat(), SIM_GetStoreLat(), true);
	BlockedMTCore->run_sim();
}

void CORE_FinegrainedMT() {
	FineGrainedCore = new Core(SIM_GetThreadsNum(), SIM_GetSwitchCycles(),
							   SIM_GetLoadLat(), SIM_GetStoreLat(), false);
	FineGrainedCore->run_sim();
}

double CORE_BlockedMT_CPI(){
	double ret = BlockedMTCore->get_cpi();
	delete BlockedMTCore;
	return ret;
}

double CORE_FinegrainedMT_CPI(){
	double ret = FineGrainedCore->get_cpi();
	delete FineGrainedCore;
	return ret;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
	BlockedMTCore->fill_thread_context(context, threadid);
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
	FineGrainedCore->fill_thread_context(context, threadid);
}
