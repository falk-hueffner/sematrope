/* sematrope - superoptimizer using the z3 SMT solver
   Copyright (C) 2018  Falk Hüffner

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include <functional>
#include <string>
#include <vector>

#include <stdint.h>

#include <z3++.h>

constexpr int REGISTER_WIDTH = 32;
constexpr int SHIFT_MASK = REGISTER_WIDTH - 1;
using uint64 = uint64_t; // needs to match z3's definition

static_assert((REGISTER_WIDTH & (REGISTER_WIDTH - 1)) == 0, "REGISTER_WIDTH must be power of two");

struct Op {
    std::string name;
    std::function<z3::expr(const z3::expr&, const z3::expr&)> eval;
};

using Opcode = int;

z3::expr bvConst(uint64 x, z3::context& c) {
    return c.bv_val(x, REGISTER_WIDTH);
}

z3::expr boolToBv(const z3::expr& b) {
    return z3::ite(b, bvConst(1, b.ctx()), bvConst(0, b.ctx()));
}

std::vector<Op> ops = {
    {"not",   [](const z3::expr& a, const z3::expr& b){ return ~a; }},
    {"neg",   [](const z3::expr& a, const z3::expr& b){ return -a; }},
    {"add",   [](const z3::expr& a, const z3::expr& b){ return a + b; }},
    {"sub",   [](const z3::expr& a, const z3::expr& b){ return a - b; }},
    {"mul",   [](const z3::expr& a, const z3::expr& b){ return a * b; }},
    {"and",   [](const z3::expr& a, const z3::expr& b){ return a & b; }},
    {"or",    [](const z3::expr& a, const z3::expr& b){ return a | b; }},
    {"xor",   [](const z3::expr& a, const z3::expr& b){ return a ^ b; }},
    {"shl",   [](const z3::expr& a, const z3::expr& b){ return z3::shl(a, b & SHIFT_MASK); }},
    {"shr",   [](const z3::expr& a, const z3::expr& b){ return z3::lshr(a, b & SHIFT_MASK); }},
    {"ashr",  [](const z3::expr& a, const z3::expr& b){ return z3::ashr(a, b & SHIFT_MASK); }},
    {"cmpeq", [](const z3::expr& a, const z3::expr& b){ return boolToBv(a == b); }},
    {"cmpne", [](const z3::expr& a, const z3::expr& b){ return boolToBv(a != b); }},
    {"cmplt", [](const z3::expr& a, const z3::expr& b){ return boolToBv(z3::ult(a, b)); }},
    {"cmpgt", [](const z3::expr& a, const z3::expr& b){ return boolToBv(z3::ugt(a, b)); }},
};

template<typename T>
std::string hex(T x) {
    std::stringstream stream;
    stream << std::hex << uint64_t(x);
    return stream.str();
}

struct Insn {
    Opcode opcode;
    int r1, r2;
    bool isImm;
    uint64 imm;

    std::string toString(int dest) const {
	std::string s = ops[opcode].name;
	s += " r" + std::to_string(r1) + ", ";
	if (isImm)
	    s += "0x" + hex(imm);
	else
	    s += "r" + std::to_string(r2);
	s += ", r" + std::to_string(dest);
	return s;
    }
};

// The z3 variables that define an instruction.
struct SymbolicInsn {
    SymbolicInsn(z3::context& c, const std::string& prefix)
	: opcode(c.int_const((prefix + "_op").c_str())),
	  r1(c.int_const((prefix + "_r1").c_str())),
	  r2(c.int_const((prefix + "_r2").c_str())),
	  imm(c.bv_const((prefix + "_imm").c_str(), REGISTER_WIDTH)) {}
    z3::expr opcode;
    // r1 is the number of a register; r2 is the number of a register
    // or implies use of the immediate if the number is out of the
    // valid range.
    z3::expr r1, r2;
    z3::expr imm;
    // We use a SSA representation where the output register is always
    // implicitly a new register, thus it doesn't need to be specified
    // here.
};

// Returns an expression representing the result of running the
// program in insns on the input value x.
z3::expr eval(const z3::expr& x, const std::vector<SymbolicInsn>& insns) {
    std::vector<z3::expr> regs;
    regs.push_back(x);
    for (std::size_t i = 0; i < insns.size(); ++i) {
	z3::expr in1 = regs[i];
	for (int j = int(i) - 1; j >= 0; --j)
	    in1 = z3::ite(insns[i].r1 == j, regs[j], in1);
	z3::expr in2 = insns[i].imm;
	for (int j = int(i); j >= 0; --j)
	    in2 = z3::ite(insns[i].r2 == j, regs[j], in2);

	z3::expr result = ops[0].eval(in1, in2);
	for (int opcode = 1; opcode < static_cast<int>(ops.size()); ++opcode)
	    result = z3::ite(insns[i].opcode == opcode, ops[opcode].eval(in1, in2), result);
	regs.push_back(result);
    }
    return regs[insns.size()];
}

std::pair<
    std::vector<SymbolicInsn>,
    std::vector<z3::expr>
    >
makeInsns(int numInsns, z3::context& c) {
    std::vector<SymbolicInsn> insns;
    std::vector<z3::expr> constraints;
    for (int i = 0; i < numInsns; ++i) {
	insns.push_back(SymbolicInsn(c, std::string("op") + std::to_string(i)));
	constraints.push_back(z3::ult(insns.back().imm, bvConst(0xff, c)));
    }
    return {insns, constraints};
}

int getIntDefault(z3::expr e, int d = 0) {
    return e.is_numeral() ? e.get_numeral_int() : d;
}
uint64 getUint64Default(z3::expr e, uint64 d = 0) {
    return e.is_numeral() ? e.get_numeral_uint64() : d;
}

std::vector<Insn> reconstructProgram(const std::vector<SymbolicInsn>& insns, const z3::model& model) {
    std::vector<Insn> result;
    for (int i = 0; i < static_cast<int>(insns.size()); ++i) {
	Insn insn;
	int opcode = getIntDefault(model.eval(insns[i].opcode), 0);
	insn.opcode = (opcode < 0 || opcode >= static_cast<int>(ops.size())) ? 0 : opcode;
	int r1 = getIntDefault(model.eval(insns[i].r1), 0);
	insn.r1 = (r1 < 0 || r1 > i) ? i : r1;
	int r2 = getIntDefault(model.eval(insns[i].r2), 0);
	if (r2 < 0 || r2 > i) {
	    insn.isImm = true;
	    insn.r2 = 0;
	    insn.imm = getUint64Default(model.eval(insns[i].imm), 0);
	} else {
	    insn.isImm = false;
	    insn.r2 = r2;
	    insn.imm = 0;
	}
	result.push_back(insn);
    }
    return result;
}

// === test cases

z3::expr isPowerOfTwoOrZero(const z3::expr& x, z3::context& c) {
    auto r = x == 0;
    uint64 p = 1;
    for (int i = 0; i < REGISTER_WIDTH; ++i) {
	r = r || (x == bvConst(p, c));
	p <<= 1;
    }
    return boolToBv(r);
}

z3::expr isPowerOfTwo(const z3::expr& x, z3::context& c) {
    z3::expr r = c.bool_val(false);
    uint64 p = 1;
    for (int i = 0; i < REGISTER_WIDTH; ++i) {
	r = r || (x == bvConst(p, c));
	p <<= 1;
    }
    return boolToBv(r);
}

z3::expr isSmallPowerOfThree(const z3::expr& x, z3::context& c) {
    z3::expr r = c.bool_val(false);
    for (uint64 p = 1; p <= 2189; p *= 3)
	r = r || (x == bvConst(p, c));
    return boolToBv(r);
}

int main() {
    const auto targetProgram = isPowerOfTwoOrZero;
    const auto inputRestriction = [](const z3::expr& x) { return x.ctx().bool_val(true); };
    //const auto targetProgram = isSmallPowerOfThree;
    //const auto inputRestriction = [](const z3::expr& x) { return z3::ule(x, 2189); };

    try {
	std::vector<uint64> testCases;
	for (int numInsns = 1; ; ++numInsns) {
	    std::cerr << "\n=== Trying with " << numInsns << " instructions ===\n\n";
	    while (true) {
		z3::context c;
		std::cerr << "Finding program with " << numInsns
			  << " instructions that is correct for all " << testCases.size() << " test cases...\n";
		auto solver = z3::solver(c);
		const auto [insns, constraints] = makeInsns(numInsns, c);
		for (const auto& c : constraints)
		    solver.add(c);
		for (const auto t : testCases) {
		    const uint64 correctResult = targetProgram(bvConst(t, c), c).simplify().get_numeral_uint64();
		    const auto programResult = eval(bvConst(t, c), insns);
		    solver.add(programResult == bvConst(correctResult, c));
		}
		const auto result = solver.check();
		if (result != z3::sat) {
		    if (result != z3::unsat)
			throw z3::exception("unexpected check value");
		    std::cerr << "Not possible to find program that is correct for all " << testCases.size() << " test cases.\n";
		    break;
		}

		std::cerr << "Found program:\n";
		const auto& model = solver.get_model();
		const auto x = c.bv_const("x", REGISTER_WIDTH);
		const auto solutionProgram = model.eval(eval(x, insns));
		const auto program = reconstructProgram(insns, model);
		for (std::size_t i = 0; i < program.size(); ++i)
		    std::cerr << program[i].toString(i + 1) << std::endl;

		std::cerr << "\nFinding counterexample...\n";
		auto cesolver = z3::solver(c);
		cesolver.add(inputRestriction(x) && solutionProgram != targetProgram(x, c));
		const auto ceResult = cesolver.check();
		if (ceResult != z3::sat) {
		    if (ceResult != z3::unsat)
			throw z3::exception("unexpected check value");
		    std::cerr << "No counterexample found. Correct program is:\n";
		    for (std::size_t i = 0; i < program.size(); ++i)
			std::cout << program[i].toString(i + 1) << std::endl;
		    return 0;
		}
		const auto& cemodel = cesolver.get_model();
		auto t = cemodel.eval(x).get_numeral_uint64();
		std::cerr << "Found counterexample: " << t
			  << " evals to " << cemodel.eval(solutionProgram).get_numeral_uint64()
			  << " but should be " << targetProgram(bvConst(t, c), c).simplify().get_numeral_uint64()
			  << std::endl;
		testCases.push_back(t);
	    }
	}
    } catch (const z3::exception& e) {
	std::cerr << e.msg() << std::endl;
	return 1;
    }

    return 0;
}
