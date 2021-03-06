/*
 *	tricore -- OTAWA loader to support TriCore with GLISS2
 *
 *	This file is part of OTAWA
 *	Copyright (c) 2011-12, IRIT UPS.
 *
 *	OTAWA is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	OTAWA is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with OTAWA; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <otawa/prog/File.h>
#include <otawa/prog/Loader.h>
#include <otawa/prog/sem.h>
#include <otawa/prog/Segment.h>
#include <otawa/hard.h>
#include <otawa/loader/gliss.h>
#include <otawa/sem/inst.h>
#include <gel/gel.h>
#include <gel/gel_elf.h>
#include <gel/debug_line.h>
extern "C" {
#	include <tricore/api.h>
#	include <tricore/config.h>
#	include <tricore/used_regs.h>
}

namespace otawa { namespace tricore {

#include "kind.h"
#include "target.h"

/****** Platform definition ******/

// registers
static hard::PlainBank regD("D", hard::Register::INT,  32, "d%d", 16);
static hard::PlainBank regA("A", hard::Register::ADDR,  32, "a%d", 16);
static hard::Register regPSW("PSW", hard::Register::BITS, 32);
static hard::Register regPC("PC", hard::Register::ADDR, 32);
static hard::Register regFCX("FCX", hard::Register::ADDR, 32);
static hard::Register regPSW_CFLAG("PSW_CFLAG", hard::Register::INT, 32);
static hard::Register regCTX("CTX", hard::Register::INT, 32);
static hard::MeltedBank misc("misc", &regPSW, &regPC,&regFCX, &regPSW_CFLAG, &regCTX, 0);

static const hard::RegBank *banks[] = {
	&regD, // 0 -15
	&regA, // 16-31
	&misc // 32 - 36
};

static const Array<const hard::RegBank *> banks_table(3, banks);


// register decoding
class RegisterDecoder {
public:
	RegisterDecoder(void) { // the used register is decided here
		
		// clear the map
		for(int i = 0; i < TRICORE_REG_COUNT; i++)
			map[i] = 0;
		
		// initialize it
		for(int i = 0; i < 16; i++) {
			map[TRICORE_REG_A(i)] = regA[i];
			map[TRICORE_REG_D(i)] = regD[i];
		}
		map[TRICORE_REG_PSW] = &regPSW;
	}

	inline hard::Register *operator[](int i) const { return map[i]; }
	
private:
	hard::Register *map[TRICORE_REG_COUNT];
};
static RegisterDecoder register_decoder;


// platform
class Platform: public hard::Platform {
public:
	static const Identification ID;

	Platform(const PropList& props = PropList::EMPTY): hard::Platform(ID, props)
		{ setBanks(banks_table); }
	Platform(const Platform& platform, const PropList& props = PropList::EMPTY)
		: hard::Platform(platform, props)
		{ setBanks(banks_table); }

	// otawa::Platform overload
	virtual bool accept(const Identification& id)
		{ return id.abi() == "eabi" && id.architecture() == "tricore"; }

	virtual const hard::Register *getSP(void) const { return regA[10]; }
	virtual const hard::Register *getPC(void) const { return misc[1]; }
};
const Platform::Identification Platform::ID("tricore-eabi-");


/****** Instruction declarations ******/

class Process;

// Inst class
class Inst: public otawa::Inst {
public:

	inline Inst(Process& process, kind_t kind, Address addr, t::uint32 size)
		: proc(process), _kind(kind), _addr(addr.offset()), isRegsDone(false), _size(size) { }

	// Inst overload
	virtual void dump(io::Output& out);
	virtual kind_t kind() { return _kind; }
	virtual address_t address() const { return _addr; }
	virtual t::uint32 size() const { return _size; }
	virtual Process &process() { return proc; }

	virtual const AllocArray<hard::Register *>& readRegs() override {
		if (!isRegsDone) {
			decodeRegs();
			isRegsDone = true;
		}
		return in_regs;
	}

	virtual const AllocArray<hard::Register *>& writtenRegs() override {
		if(!isRegsDone) {
			decodeRegs();
			isRegsDone = true;
		}
		return out_regs;
	}
	
	virtual void semInsts(otawa::sem::Block &block);
	virtual void semKernel(otawa::sem::Block &block);

	
protected:
	Process &proc;

private:
	void decodeRegs(void);
	kind_t _kind;
	AllocArray<hard::Register *> in_regs;
	AllocArray<hard::Register *> out_regs;
	tricore_address_t _addr;
	bool isRegsDone;
	t::uint32 _size;
};


// BranchInst class
class BranchInst: public Inst {
public:

	inline BranchInst(Process& process, kind_t kind, Address addr, t::uint32 size)
		: Inst(process, kind, addr, size), _target(0), isTargetDone(false)
		{ }

	virtual otawa::Inst *target();

protected:
	tricore_address_t decodeTargetAddress(void);

private:
	otawa::Inst *_target;
	bool isTargetDone;
};



/****** Segment class ******/
class Segment: public otawa::Segment {
public:
	Segment(Process& process,
		CString name,
		address_t address,
		size_t size,
		flags_t flags)
	: otawa::Segment(name, address, size, flags), proc(process) { }

protected:
	virtual otawa::Inst *decode(address_t address);

private:
	Process& proc;
};


/****** Process class ******/

class Process: public otawa::Process, public gliss::Info {
public:

	Process(Manager *manager, hard::Platform *pf, const PropList& props = PropList::EMPTY)
	:	otawa::Process(manager, props),
	 	_start(0),
	 	oplatform(pf),
		_memory(0),
		init(false),
		map(0),
		no_stack(true),
		_file(nullptr)
	{
		ASSERTP(manager, "manager required");
		ASSERTP(pf, "platform required");

		// gliss2 ppc structs
		_platform = tricore_new_platform();
		ASSERTP(_platform, "cannot create an arm_platform");
		_decoder = tricore_new_decoder(_platform);
		ASSERTP(_decoder, "cannot create an arm_decoder");
		_memory = tricore_get_memory(_platform, TRICORE_MAIN_MEMORY);
		ASSERTP(_memory, "cannot get main arm_memory");
		tricore_lock_platform(_platform);

		// build arguments
		static char no_name[1] = { 0 };
		static char *default_argv[] = { no_name, 0 };
		static char *default_envp[] = { 0 };
		argc = ARGC(props);
		if (argc < 0)
			argc = 1;
		argv = ARGV(props);
		if (!argv)
			argv = default_argv;
		else
			no_stack = false;
		envp = ENVP(props);
		if (!envp)
			envp = default_envp;
		else
			no_stack = false;

		// handle features
		provide(MEMORY_ACCESS_FEATURE);
		provide(SOURCE_LINE_FEATURE);
		provide(CONTROL_DECODING_FEATURE);
		provide(REGISTER_USAGE_FEATURE);
		provide(MEMORY_ACCESSES);
		provide(gliss::INFO_FEATURE);
		gliss::INFO(this) = this;
	}

	virtual ~Process() {
		tricore_delete_decoder(_decoder);
		tricore_unlock_platform(_platform);
		if(_file)
			gel_close(_file);
	}

	// gliss::Info overload
	virtual bool check(t::uint32 checksum) { return true; }
	virtual void *decode(otawa::Inst *inst) { return decode_raw(inst->address()); }
	virtual void free(void *desc) { release((tricore_inst_t *)desc); }

	// Process overloads
	virtual int instSize(void) const { return 0; }
	virtual hard::Platform *platform(void) { return oplatform; }
	virtual otawa::Inst *start(void) { return _start; }

	virtual File *loadFile(elm::CString path) {

		// check if there is not an already opened file !
		if(program())
			throw LoadException("loader cannot open multiple files !");

		// make the file
		File *file = new otawa::File(path);
		addFile(file);

		// build the environment
		gel_env_t genv = *gel_default_env();
		genv.argv = argv;
		genv.envp = envp;
		if(no_stack)
			genv.flags = GEL_ENV_NO_STACK;

		// build the GEL image
		_file = gel_open(path.chars(), NULL, 0);
		if(!_file)
			throw LoadException(_ << "cannot load \"" << path << "\": " << gel_strerror());
		gel_image_t *gimage = gel_image_load(_file, &genv, 0);
		if(!gimage) {
			gel_close(_file);
			throw LoadException(_ << "cannot build image of \"" << path << "\": " << gel_strerror());
		}

		// build the GLISS image
		gel_image_info_t iinfo;
		gel_image_infos(gimage, &iinfo);
		for(int i = 0; i < iinfo.membersnum; i++) {
			gel_cursor_t cursor;
			gel_block2cursor(iinfo.members[i], &cursor);
			tricore_mem_write(_memory,
				gel_cursor_vaddr(cursor),
				gel_cursor_addr(&cursor),
				gel_cursor_avail(cursor));
			//cerr << "writing in memory at " << Address(gel_cursor_vaddr(cursor)) << ":" << io::hex(gel_cursor_avail(cursor)) << io::endl;
		}

		// cleanup image
		gel_image_close(gimage);

		// build segments
		gel_file_info_t infos;
		gel_file_infos(_file, &infos);
		for (int i = 0; i < infos.sectnum; i++) {
			gel_sect_info_t infos;
			gel_sect_t *sect = gel_getsectbyidx(_file, i);
			assert(sect);
			gel_sect_infos(sect, &infos);
			if(infos.vaddr != 0 && infos.size != 0) {
				Segment::flags_t flags = 0;
				if((infos.flags & SHF_WRITE) != 0)
					flags |= Segment::WRITABLE;
				if((infos.flags & SHF_EXECINSTR) != 0)
					flags |= Segment::EXECUTABLE;
				if((infos.flags & SHF_ALLOC) != 0)
					flags |= Segment::INITIALIZED;
				Segment *seg = new Segment(*this, infos.name, infos.vaddr, infos.size, flags);
				file->addSegment(seg);
			}
		}

		// Initialize symbols
		gel_enum_t *iter = gel_enum_file_symbol(_file);
		gel_enum_initpos(iter);
		for(char *name = (char *)gel_enum_next(iter); name; name = (char *)gel_enum_next(iter)) {
			ASSERT(name);
			if(string(name).startsWith(".L"))
				continue;
			Address addr = Address::null;
			Symbol::kind_t kind;
			gel_sym_t *sym = gel_find_file_symbol(_file, name);
			assert(sym);
			gel_sym_info_t infos;
			gel_sym_infos(sym, &infos);

			if(infos.sect == SHN_ABS) // absolute value is not useful for the symbol
				continue;

			switch(ELF32_ST_TYPE(infos.info)) {
			case STT_FUNC:
				kind = Symbol::FUNCTION;
				addr = Address(infos.vaddr);
				break;
			case STT_NOTYPE:
				kind = Symbol::LABEL;
				addr = Address(infos.vaddr);
				break;
			default:
				continue;
			}

			// Build the label if required
			if(addr != Address::null) {
				String label(infos.name);
				Symbol *sym = new Symbol(*file, label, kind, addr, infos.size);
				file->addSymbol(sym);
			}
		}
		gel_enum_free(iter);

		// Last initializations
		_start = findInstAt((address_t)infos.entry);
		return file;
	}

	// internal work
	void decodeRegs(Inst *oinst,
		AllocArray<hard::Register *>& in,
		AllocArray<hard::Register *>& out)
	{
		// Decode instruction
		tricore_inst_t *inst = decode_raw(oinst->address());
		if(inst->ident == TRICORE_UNKNOWN) {
			free(inst);
			return;
		}

		// get register infos
		tricore_used_regs_read_t rds;
		tricore_used_regs_write_t wrs;
		Vector<hard::Register *> reg_in;
		Vector<hard::Register *> reg_out;
		tricore_used_regs(inst, rds, wrs);
		for (int i = 0; rds[i] != -1; i++ ) {
			hard::Register *r = register_decoder[rds[i]];
			if(r)
				reg_in.add(r);
		}
		for (int i = 0; wrs[i] != -1; i++ ) {
			hard::Register *r = register_decoder[wrs[i]];
			if(r)
				reg_out.add(r);
		}

		// store results
		in = AllocArray<hard::Register *>(reg_in.length());
		for(int i = 0 ; i < reg_in.length(); i++)
			in.set(i, reg_in.get(i));
		out = AllocArray<hard::Register *>(reg_out.length());
		for (int i = 0 ; i < reg_out.length(); i++)
			out.set(i, reg_out.get(i));

		// Free instruction
		free(inst);
	}

	otawa::Inst *decode(Address addr) {
		tricore_inst_t *inst = decode_raw(addr);

		// compute the description of the instruction
		Inst::kind_t kind = 0;
		otawa::Inst *result = 0;
		kind = tricore_kind(inst);
		t::uint32 size = tricore_get_inst_size(inst) / 8;

		// unknown instruction: 16-bits?
		if(inst->ident == TRICORE_UNKNOWN) {
			t::uint8 b;
			get(Address(inst->addr), b);
			if(!(b & 0x1))
				size = 2;
		}

		// build the instruction
		if(kind & Inst::IS_CONTROL)
			result = new BranchInst(*this, kind, addr, size);
		else
			result = new Inst(*this, kind, addr, size);

		// cleanup
		free(inst);
		return result;
	}


	inline int opcode(Inst *inst) const {
		tricore_inst_t *i = decode_raw(inst->address());
		int code = i->ident;
		release(i);
		return code;
	}

	inline ::tricore_inst_t *decode_raw(Address addr) const
		{ return tricore_decode(decoder(), ::tricore_address_t(addr.offset())); }

	inline void release(tricore_inst_t *inst) const { tricore_free_inst(inst); }
	virtual gel_file_t *file(void) const { return _file; }
	virtual tricore_memory_t *memory(void) const { return _memory; }
	inline tricore_decoder_t *decoder() const { return _decoder; }
	inline void *platform(void) const { return _platform; }

	virtual Option<Pair<cstring, int> > getSourceLine(Address addr) override {
		setup_debug();
		if (!map)
			return none;
		const char *file;
		int line;
		if (!map || gel_line_from_address(map, addr.offset(), &file, &line) < 0)
			return none;
		return some(pair(cstring(file), line));
	}

	virtual void getAddresses(cstring file, int line, Vector<Pair<Address, Address> >& addresses) override {
		setup_debug();
		addresses.clear();
		if (!map)
			return;
		gel_line_iter_t iter;
		gel_location_t loc, ploc = { 0, 0, 0, 0 };
		for (loc = gel_first_line(&iter, map); loc.file; loc = gel_next_line(&iter))
		{
			cstring lfile = loc.file;
			if (file == loc.file || lfile.endsWith(file)) {
				if (line == loc.line)
					addresses.add(pair(Address(loc.low_addr), Address(loc.high_addr)));
				else if(loc.file == ploc.file && line > ploc.line && line < loc.line)
					addresses.add(pair(Address(ploc.low_addr), Address(ploc.high_addr)));
			}
			ploc = loc;
		}
	}


	virtual void get(Address at, t::int8& val) {
		val = tricore_mem_read8(_memory, at.offset());
	}
	virtual void get(Address at, t::uint8& val) {
		val = tricore_mem_read8(_memory, at.offset());
	}
	virtual void get(Address at, t::int16& val) {
		unsigned long val1 = tricore_mem_read8(_memory, at.offset());
		unsigned long val2 = tricore_mem_read8(_memory, at.offset() + 1);
		val = val2 << 8 | val1;
	}
	virtual void get(Address at, t::uint16& val) {
		unsigned long val1 = tricore_mem_read8(_memory, at.offset());
		unsigned long val2 = tricore_mem_read8(_memory, at.offset() + 1);
		val = val2 << 8 | val1;
	}
	virtual void get(Address at, t::int32& val) {
		unsigned long val1 = tricore_mem_read8(_memory, at.offset());
		unsigned long val2 = tricore_mem_read8(_memory, at.offset() + 1);
		unsigned long val3 = tricore_mem_read8(_memory, at.offset() + 2);
		unsigned long val4 = tricore_mem_read8(_memory, at.offset() + 3);
		val = (val4 << 24) | (val3 << 16) | (val2 << 8) | val1;
	}
	virtual void get(Address at, t::uint32& val) {
		unsigned long val1 = tricore_mem_read8(_memory, at.offset());
		unsigned long val2 = tricore_mem_read8(_memory, at.offset() + 1);
		unsigned long val3 = tricore_mem_read8(_memory, at.offset() + 2);
		unsigned long val4 = tricore_mem_read8(_memory, at.offset() + 3);
		val = (val4 << 24) | (val3 << 16) | (val2 << 8) | val1;
	}
	virtual void get(Address at, t::int64& val) {
		unsigned long val1 = tricore_mem_read8(_memory, at.offset());
		unsigned long val2 = tricore_mem_read8(_memory, at.offset() + 1);
		unsigned long val3 = tricore_mem_read8(_memory, at.offset() + 2);
		unsigned long val4 = tricore_mem_read8(_memory, at.offset() + 3);
		unsigned long val5 = tricore_mem_read8(_memory, at.offset() + 4);
		unsigned long val6 = tricore_mem_read8(_memory, at.offset() + 5);
		unsigned long val7 = tricore_mem_read8(_memory, at.offset() + 6);
		unsigned long val8 = tricore_mem_read8(_memory, at.offset() + 7);
		val = (val8 << 56) | (val7 << 48) | (val6 << 40) | (val5 << 32) | (val4 << 24) | (val3 << 16) | (val2 << 8) | val1;
	}
	virtual void get(Address at, t::uint64& val) {
		unsigned long val1 = tricore_mem_read8(_memory, at.offset());
		unsigned long val2 = tricore_mem_read8(_memory, at.offset() + 1);
		unsigned long val3 = tricore_mem_read8(_memory, at.offset() + 2);
		unsigned long val4 = tricore_mem_read8(_memory, at.offset() + 3);
		unsigned long val5 = tricore_mem_read8(_memory, at.offset() + 4);
		unsigned long val6 = tricore_mem_read8(_memory, at.offset() + 5);
		unsigned long val7 = tricore_mem_read8(_memory, at.offset() + 6);
		unsigned long val8 = tricore_mem_read8(_memory, at.offset() + 7);
		val = (val8 << 56) | (val7 << 48) | (val6 << 40) | (val5 << 32) | (val4 << 24) | (val3 << 16) | (val2 << 8) | val1;
	}
	virtual void get(Address at, Address& val)
		{ val = tricore_mem_read32(_memory, at.offset()); }
	virtual void get(Address at, string& str) {
		Address base = at;
		while(!tricore_mem_read8(_memory, at.offset()))
			at = at + 1;
		int len = at - base;
		char buf[len];
		get(base, buf, len);
		str = String(buf, len);
	}
	virtual void get(Address at, char *buf, int size)
		{ tricore_mem_read(_memory, at.offset(), buf, size); }

private:
	void setup_debug(void) {
		ASSERT(_file);
		if(init)
			return;
		init = true;
		map = gel_new_line_map(_file);
	}

	otawa::Inst *_start;
	hard::Platform *oplatform;
	tricore_platform_t *_platform;
	tricore_memory_t *_memory;
	tricore_decoder_t *_decoder;
	gel_line_map_t *map;
	gel_file_t *_file;
	int argc;
	char **argv, **envp;
	bool no_stack;
	bool init;
};


/****** Instructions implementation ******/

void Inst::dump(io::Output& out) {
	char out_buffer[200];
	tricore_inst_t *inst = proc.decode_raw(_addr);
	tricore_disasm(out_buffer, inst);
	proc.release(inst);
	out << out_buffer;
}

void Inst::decodeRegs(void) {
	proc.decodeRegs(this, in_regs, out_regs);
}


tricore_address_t BranchInst::decodeTargetAddress(void) {
	tricore_inst_t *inst= proc.decode_raw(address());
	tricore_address_t target_addr = tricore_target(inst);
	proc.release(inst);
	return target_addr;
}


otawa::Inst *BranchInst::target() {
	if (!isTargetDone) {
		tricore_address_t a = decodeTargetAddress();
		if (a)
			_target = process().findInstAt(a);
		isTargetDone = true;
	}
	return _target;
}


otawa::Inst *Segment::decode(address_t address) {
	return proc.decode(address);
}


static void tricore_sem(tricore_inst_t *inst, otawa::sem::Block& block); // @suppress("Unused function declaration")


/**
 */
void Inst::semInsts (otawa::sem::Block &block) {

	// get the block
	tricore_inst_t *inst = proc.decode_raw(address());
	if(inst->ident == TRICORE_UNKNOWN)
		return;
	tricore_sem(inst, block);
	tricore_free_inst(inst);

//	// fix spurious instructions possibly generated with conditional instructions
//	for(int i = 0; i < block.length(); i++)
//		if(block[i].op == otawa::sem::CONT) {
//			block.setLength(i);
//			break;
//		}
}


/**
 */
void Inst::semKernel(otawa::sem::Block &block) {
	tricore_inst_t *inst = proc.decode_raw(address());
	if(inst->ident == TRICORE_UNKNOWN)
		return;
	tricore_sem(inst, block);
	tricore_free_inst(inst);
}


/* semantics support */

#define D(n)		regD[n]->platformNumber()
#define A(n)		regA[n]->platformNumber()
#define P(n)		A(n)
#define _PSW		regPSW.platformNumber()
#define FCX			regFCX.platformNumber()
#define PC			regPC.platformNumber()
#define _PSW_CFLAG regPSW_CFLAG.platformNumber()
#define _CTX		regCTX.platformNumber()
#define A10			A(10)
#define A11			A(11)
#define A15			A(15)
#define D15			D(15)

#define T1			(-1)
#define T2			(-2)
#define T3			(-3)
#define T4			(-4)

#define IADDR		(inst->address().offset())

#define NE			sem::NE
#define EQ			sem::EQ
#define LT			sem::LT
#define LE			sem::LE
#define GE			sem::GE
#define GT			sem::GT
#define ULT			sem::ULT
#define UGE			sem::UGE

#define SCRATCH(a)		block.add(sem::scratch(a))
#define SET(a, b)			block.add(sem::set(a, b))
#define SETI(a, b)		block.add(sem::seti(a, b))
#define ADD(a, b, c)	block.add(sem::add(a, b, c))
#define SUB(a, b, c)	block.add(sem::sub(a, b, c))
#define SHL(a, b, c)	block.add(sem::shl(a, b, c))
#define SHR(a, b, c)	block.add(sem::shr(a, b, c))
#define CMP(a, b, c)	block.add(sem::cmp(a, b, c))
#define CMPU(a, b, c)	block.add(sem::cmpu(a, b, c))
#define IF(a, b, c)		block.add(sem::_if(a, b, c))
#define SCRATCH8(a)		SCRATCH(a); SCRATCH(a + 1)
#define BRANCH(a)			block.add(sem::branch(a))
#define CONT()					block.add(sem::cont())
#define NOP()			block.add(sem::nop())
#define LOADW(d, a)		block.add(sem::load(d, a, sem::INT32))
#define LOADD(d, a)		block.add(sem::load(d, a, sem::INT64))
#define LOADSB(d, a)	block.add(sem::load(d, a, sem::INT8))
#define LOADUB(d, a)	block.add(sem::load(d, a, sem::UINT8))
#define LOADSH(d, a)	block.add(sem::load(d, a, sem::INT16))
#define LOADUH(d, a)	block.add(sem::load(d, a, sem::UINT16))
#define STOREW(d, a)	block.add(sem::store(d, a, sem::INT32))
#define STORED(d, a)	block.add(sem::store(d, a, sem::INT64))
#define STOREB(d, a)	block.add(sem::store(d, a, sem::INT8))
#define STOREH(d, a)	block.add(sem::store(d, a, sem::INT16))
#define TRAP(c)				block.add(sem::trap(c))
#define ASR(a,b,c)		block.add(sem::asr(a,b,c))
#define NEG(d,a)			block.add(sem::neg(d,a))
#define NOT(d,a)			block.add(sem::_not(d,a))
#define AND(d,a,b)		block.add(sem::_and(d,a,b))
#define OR(d,a,b)			block.add(sem::_or(d,a,b))
#define XOR(d,a,b)		block.add(sem::_xor(d,a,b))
#define MUL(d,a,b)		block.add(sem::mul(d,a,b))
#define MULU(d,a,b)		block.add(sem::mulu(d,a,b))
#define DIV(d,a,b)		block.add(sem::div(d,a,b))
#define DIVU(d,a,b)		block.add(sem::divu(d,a,b))
#define MOD(d,a,b)		block.add(sem::mod(d,a,b))
#define MODU(d,a,b)		block.add(sem::modu(d,a,b))

#define E(n)			D(n)
#define S4(n)		(((int32_t)instr->instrinput[n].val.uint8 << 28) >> 28)
#define S6(n)		(((int8_t)instr->instrinput[n].val.uint8 << 2) >> 2)
#define S8(n)		(instr->instrinput[n].val.int8)
#define S9(n)		(((int32_t)instr->instrinput[n].val.uint16 << 23) >> 23)
#define S15(n)		(((int32_t)instr->instrinput[n].val.uint16 << 17) >> 17)
#define S16(n)		instr->instrinput[n].val.int16
#define U2(n)		instr->instrinput[n].val.uint8
#define U4(n)		instr->instrinput[n].val.uint8
#define U6(n)		instr->instrinput[n].val.uint8
#define U8(n)		instr->instrinput[n].val.uint8
#define U16(n)		instr->instrinput[n].val.uint16
#define SAVE_CONTEXT	\
		SETI(T1, FCX); SETI(T2, 4); LOADW(T3, T1); \
		STOREW(FCX, T1); ADD(T1, T1, T2); \
		STOREW(PSW, T1); ADD(T1, T1, T2); \
		STOREW(A(10), T1); ADD(T1, T1, T2); \
		STOREW(A(11), T1); ADD(T1, T1, T2); \
		STOREW(D(8), T1); ADD(T1, T1, T2); \
		STOREW(D(9), T1); ADD(T1, T1, T2); \
		STOREW(D(10), T1); ADD(T1, T1, T2); \
		STOREW(D(11), T1); ADD(T1, T1, T2); \
		STOREW(A(12), T1); ADD(T1, T1, T2); \
		STOREW(A(13), T1); ADD(T1, T1, T2); \
		STOREW(A(14), T1); ADD(T1, T1, T2); \
		STOREW(A(15), T1); ADD(T1, T1, T2); \
		STOREW(D(12), T1); ADD(T1, T1, T2); \
		STOREW(D(13), T1); ADD(T1, T1, T2); \
		STOREW(D(14), T1); ADD(T1, T1, T2); \
		STOREW(D(15), T1); \
		SETI(T1, 1 << 6); ADD(FCX, FCX, T1)
#define LOAD_CONTEXT	\
		SETI(T1, FCX); SETI(T2, 4); \
		LOADW(T3, T1); ADD(T1, T1, T2); \
		LOADW(PSW, T1); ADD(T1, T1, T2); \
		LOADW(A(10), T1); ADD(T1, T1, T2); \
		LOADW(A(11), T1); ADD(T1, T1, T2); \
		LOADW(D(8), T1); ADD(T1, T1, T2);  \
		LOADW(D(9), T1); ADD(T1, T1, T2);  \
		LOADW(D(10), T1); ADD(T1, T1, T2); \
		LOADW(D(11), T1); ADD(T1, T1, T2); \
		LOADW(A(12), T1); ADD(T1, T1, T2); \
		LOADW(A(13), T1); ADD(T1, T1, T2); \
		LOADW(A(14), T1); ADD(T1, T1, T2); \
		LOADW(A(15), T1); ADD(T1, T1, T2); \
		LOADW(D(12), T1); ADD(T1, T1, T2); \
		LOADW(D(13), T1); ADD(T1, T1, T2); \
		LOADW(D(14), T1); ADD(T1, T1, T2); \
		LOADW(D(15), T1); \
		STOREW(FCX, T1); SET(FCX, T3)
#define INT10(d, f4, f6)	SETI(d, (S4(f4) << 6) | U6(f6))
#define INT18(d, f0, f1, f2, f3)	SETI(d, (U4(f0) << 28) | (U4(f1) << 10) | (U4(f2) << 6) | U6(f3))
#define INT16(d, f0, f1, f2)		SETI(d, (S6(f0)<< 10) | (U4(f1) << 6) | U6(f2))

#include "sem.h"


/****** loader definition ******/

// loader definition
class Loader: public otawa::Loader {
public:
	Loader(void): otawa::Loader(make("tricore", OTAWA_LOADER_VERSION)
		.version("2.0.0")
		.description("loader for TriCore architecture")
		.license(Manager::copyright)
		.alias("elf_44")) { }

	virtual otawa::Process *load(Manager *man, CString path, const PropList& props) {
		otawa::Process *proc = create(man, props);
		if (!proc->loadProgram(path)) {
			delete proc;
			return 0;
		}
		else
			return proc;
	}

	virtual cstring getName(void) const { return "tricore"; }

	virtual otawa::Process *create(Manager *man, const PropList& props) {
		return new Process(man, new Platform(props), props);
	}
};


} }		// otawa::tricore

// hooks
otawa::tricore::Loader OTAWA_LOADER_HOOK;
otawa::tricore::Loader& tricore_plugin = OTAWA_LOADER_HOOK;

