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

#include "targets/isPowerOfTwo.hh"

#include "sematrope.hh"

#include <functional>
#include <string>
#include <vector>

#include <stdint.h>

#include <z3++.h>

std::vector<Target>& getTargets() {
    static std::vector<Target> targets;
    return targets;
}

void registerTarget(const Target& target) {
    getTargets().push_back(target);
}

z3::context context;
int registerWidth;
int shiftMask;

// Return a constant with the same context and width as variable x.
z3::expr bvConst(uint64_t x) {
    return context.bv_val(x, registerWidth);
}

z3::expr boolToBv(const z3::expr& b) {
    return z3::ite(b, bvConst(1), bvConst(0));
}

struct Op {
    std::string name;
    std::function<z3::expr(const z3::expr&, const z3::expr&)> eval;
};

using Opcode = int;

std::vector<Op> ops = {
    {"not",   [](const z3::expr& a, const z3::expr&){ return ~a; }},
    {"neg",   [](const z3::expr& a, const z3::expr&){ return -a; }},
    {"add",   [](const z3::expr& a, const z3::expr& b){ return a + b; }},
    {"sub",   [](const z3::expr& a, const z3::expr& b){ return a - b; }},
    {"mul",   [](const z3::expr& a, const z3::expr& b){ return a * b; }},
    {"div",   [](const z3::expr& a, const z3::expr& b){ return a * b; }},
    {"and",   [](const z3::expr& a, const z3::expr& b){ return a & b; }},
    {"or",    [](const z3::expr& a, const z3::expr& b){ return a | b; }},
    {"xor",   [](const z3::expr& a, const z3::expr& b){ return a ^ b; }},
    {"shl",   [](const z3::expr& a, const z3::expr& b){ return z3::shl(a, b & shiftMask); }},
    {"shr",   [](const z3::expr& a, const z3::expr& b){ return z3::lshr(a, b & shiftMask); }},
    {"ashr",  [](const z3::expr& a, const z3::expr& b){ return z3::ashr(a, b & shiftMask); }},
    {"cmpeq", [](const z3::expr& a, const z3::expr& b){ return boolToBv(a == b); }},
    {"cmpne", [](const z3::expr& a, const z3::expr& b){ return boolToBv(a != b); }},
    {"cmplt", [](const z3::expr& a, const z3::expr& b){ return boolToBv(a < b); }},
    {"cmpgt", [](const z3::expr& a, const z3::expr& b){ return boolToBv(a > b); }},
    {"cmple", [](const z3::expr& a, const z3::expr& b){ return boolToBv(a <= b); }},
    {"cmpge", [](const z3::expr& a, const z3::expr& b){ return boolToBv(a >= b); }},
    {"cmpult", [](const z3::expr& a, const z3::expr& b){ return boolToBv(z3::ult(a, b)); }},
    {"cmpugt", [](const z3::expr& a, const z3::expr& b){ return boolToBv(z3::ugt(a, b)); }},
    {"cmpule", [](const z3::expr& a, const z3::expr& b){ return boolToBv(z3::ule(a, b)); }},
    {"cmpuge", [](const z3::expr& a, const z3::expr& b){ return boolToBv(z3::uge(a, b)); }},
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
    uint64_t imm;

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
	  imm(c.bv_const((prefix + "_imm").c_str(), registerWidth)) {}
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
z3::expr eval(const std::vector<z3::expr>& input, const std::vector<SymbolicInsn>& insns) {
    std::vector<z3::expr> regs;
    for (const auto& x : input)
	regs.push_back(x);
    for (std::size_t i = 0; i < insns.size(); ++i) {
       z3::expr in1 = regs[0];
       for (int j = 1; j < static_cast<int>(regs.size()); ++j)
	    in1 = z3::ite(insns[i].r1 == j, regs[j], in1);
       z3::expr in2 = insns[i].imm;
       for (int j = 0; j < static_cast<int>(regs.size()); ++j)
	   in2 = z3::ite(insns[i].r2 == j, regs[j], in2);

	z3::expr result = ops[0].eval(in1, in2);
	for (int opcode = 1; opcode < static_cast<int>(ops.size()); ++opcode)
	    result = z3::ite(insns[i].opcode == opcode, ops[opcode].eval(in1, in2), result);
	regs.push_back(result);
    }
    return regs.back();
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
	constraints.push_back(z3::ult(insns.back().imm, bvConst(0xff)));
    }
    return {insns, constraints};
}

int getIntDefault(z3::expr e, int d = 0) {
    return e.is_numeral() ? e.get_numeral_int() : d;
}
uint64_t getUint64Default(z3::expr e, uint64_t d = 0) {
    return e.is_numeral() ? e.get_numeral_uint64() : d;
}

std::vector<Insn> reconstructProgram(const std::vector<SymbolicInsn>& insns,
				     const z3::model& model,
				     const int numInputRegisters) {
    std::vector<Insn> result;
    for (int i = 0; i < static_cast<int>(insns.size()); ++i) {
	Insn insn;
	int opcode = getIntDefault(model.eval(insns[i].opcode), 0);
	insn.opcode = (opcode < 0 || opcode >= static_cast<int>(ops.size())) ? 0 : opcode;
	int r1 = getIntDefault(model.eval(insns[i].r1), 0);
	insn.r1 = (r1 < 0 || r1 >= numInputRegisters + i) ? 0 : r1;
	int r2 = getIntDefault(model.eval(insns[i].r2), 0);
	const bool outOfRange = r2 < 0 || r2 >= numInputRegisters + i;
	if (outOfRange) {
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

int main() {
    const auto targetName = "isPowerOfTwoNonzero";
    Target target;
    for (const auto& t : getTargets()) {
	if (t.name == targetName) {
	    target = t;
	    break;
	}
    }
    assert(!target.name.empty());
    registerWidth = target.registerWidth;
    assert((registerWidth & (registerWidth - 1)) == 0);
    shiftMask = registerWidth - 1;

    std::cerr << "=== Finding optimal program for " << target.name << " ===\n";
    try {
	std::vector<std::vector<uint64_t>> testCases;
	for (int numInsns = 1; ; ++numInsns) {
	    std::cerr << "\n=== Trying with " << numInsns << " instructions ===\n\n";
	    while (true) {
		std::cerr << "Finding program with " << numInsns
			  << " instructions that is correct for all " << testCases.size() << " test cases...\n";
		auto solver = z3::solver(context);
		const auto [insns, constraints] = makeInsns(numInsns, context);
		for (const auto& constraint : constraints)
		    solver.add(constraint);
		for (const auto& t : testCases) {
		    std::vector<z3::expr> inputs;
		    for (auto x : t)
			inputs.push_back(bvConst(x));
		    const uint64_t correctResult = target.target(inputs).simplify().get_numeral_uint64();
		    const auto programResult = eval(inputs, insns);
		    solver.add(programResult == bvConst(correctResult));
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
		std::vector<z3::expr> inputs;
		for (int i = 0; i < target.arity; ++i)
		    inputs.push_back(context.bv_const(("x" + std::to_string(i)).c_str(), target.registerWidth));
		const auto solutionProgram = model.eval(eval(inputs, insns));
		const auto program = reconstructProgram(insns, model, target.arity);
		for (std::size_t i = 0; i < program.size(); ++i)
		    std::cerr << program[i].toString(i + target.arity) << std::endl;

		std::cerr << "\nFinding counterexample...\n";
		auto cesolver = z3::solver(context);
		cesolver.add(target.isValidInput(inputs) && solutionProgram != target.target(inputs));
		const auto ceResult = cesolver.check();
		if (ceResult != z3::sat) {
		    if (ceResult != z3::unsat)
			throw z3::exception("unexpected check value");
		    std::cerr << "No counterexample found. Correct program is:\n";
		    for (std::size_t i = 0; i < program.size(); ++i)
			std::cout << program[i].toString(i + target.arity) << std::endl;
		    return 0;
		}
		const auto& cemodel = cesolver.get_model();
		std::vector<uint64_t> t;
		for (int i = 0; i < static_cast<int>(inputs.size()); ++i) {
		    const auto& v = cemodel.eval(inputs[i]);
		    t.push_back(v.is_numeral() ? v.get_numeral_uint64() : 0);
		}
		std::vector<z3::expr> z3t;
		for (auto x : t)
		    z3t.push_back(bvConst(x));

		std::cerr << "Found counterexample: [ ";
		for (auto x : t)
		    std::cerr << x << ' ';
		std::cerr << "] evals to " << cemodel.eval(solutionProgram).get_numeral_uint64()
			  << " but should be " << target.target(z3t).simplify().get_numeral_uint64()
			  << std::endl;
		testCases.push_back(t);
	    }
	}
    } catch (const z3::exception& e) {
	std::cerr << e.msg() << std::endl;
	return 1;
    }
}
